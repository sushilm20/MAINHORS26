package org.firstinspires.ftc.teamcode.tracking;

import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.function.BooleanSupplier;

/**
 * TurretGoalAimer - Pose-Based Point Tracking with Homing Sweep
 *
 * Features:
 * - Aims turret at fixed field point using robot pose (X, Y, heading)
 * - Homing sweep to find magnetic limit switch
 * - Freeze/hold mode after homing
 * - Manual override support
 */
public class TurretGoalAimer {

    // Hardware
    private final DcMotor turretMotor;
    private final BNO055IMU imu;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;

    // Optional reset trigger (magnetic limit switch)
    private BooleanSupplier encoderResetTrigger = null;

    // Target pose
    private Pose targetPose = new Pose(14, 134, 0);

    // Turret encoder hard limits (configurable)
    @Sorter(sort = 0)
    public static int TURRET_MIN_POS = -1000;
    @Sorter(sort = 1)
    public static int TURRET_MAX_POS = 1000;
    @Sorter(sort = 2)
    public static double TICKS_PER_RADIAN_SCALE = 0.87;

    // PID like config
    @Sorter(sort = 3)
    public static double TURRET_KP = 1.0;
    @Sorter(sort = 4)
    public static double TURRET_KI = 0.0;
    @Sorter(sort = 5)
    public static double TURRET_KD = 0.235;
    @Sorter(sort = 6)
    public static double TURRET_MAX_POWER = 1.0;

    // Feedforward and smoothing/filtering (configurable)
    @Sorter(sort = 7)
    public static double FF_GAIN = 5.0;
    @Sorter(sort = 8)
    public static double POWER_SMOOTH_ALPHA = 0.9;
    @Sorter(sort = 9)
    public static double DERIV_FILTER_ALPHA = 1.0;

    // Deadband & anti-windup (configurable)
    @Sorter(sort = 10)
    public static int SMALL_DEADBAND_TICKS = 3;
    @Sorter(sort = 11)
    public static double INTEGRAL_CLAMP = 50.0;

    // Homing sweep configuration
    public static final int HOMING_AMPLITUDE_TICKS = 300;
    public static final double HOMING_POWER = 0.5;
    public static final int HOMING_TARGET_DEADBAND = 12;
    public static final long HOMING_TIMEOUT_MS = 3000;

    // PID state
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Reference state
    private double headingReferenceRad = 0.0;
    private double lastHeadingRad = 0.0;
    private int turretEncoderReference = 0;
    private boolean manualActiveLast = false;

    // Homing sweep state
    private boolean homingMode = false;
    private boolean homingCommandPrev = false;
    private boolean homingDirectionPos = true;
    private int homingTarget = HOMING_AMPLITUDE_TICKS;
    private long homingStartMs = 0L;

    // Freeze/hold mode after homing
    private boolean freezeMode = false;
    private int freezeHoldTarget = 0;

    // Telemetry
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;
    private double lastBearingDeg = 0.0;
    private double lastRobotRelativeAngleDeg = 0.0;

    public TurretGoalAimer(DcMotor turretMotor, BNO055IMU imu, GoBildaPinpointDriver pinpoint, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
        captureReferences();
        resetPidState();
    }

    public TurretGoalAimer(DcMotor turretMotor, BNO055IMU imu, Telemetry telemetry) {
        this(turretMotor, imu, null, telemetry);
    }

    /**
     * Set encoder reset trigger (magnetic limit switch)
     * Example: turretAimer.setEncoderResetTrigger(() -> !limitSwitch.getState());
     */
    public void setEncoderResetTrigger(BooleanSupplier trigger) {
        this.encoderResetTrigger = trigger;
    }

    /**
     * Command homing sweep - oscillate to find limit switch
     */
    public void commandHomingSweep(boolean homingButtonPressed) {
        if (homingButtonPressed && !homingCommandPrev) {
            if (freezeMode) {
                // Unfreeze: resume tracking
                freezeMode = false;
                captureReferences();
                resetPidState();
                lastTimeMs = System.currentTimeMillis();
            } else {
                startHomingSweep();
            }
        }
        homingCommandPrev = homingButtonPressed;
    }

    private void startHomingSweep() {
        homingMode = true;
        homingDirectionPos = true;
        homingTarget = HOMING_AMPLITUDE_TICKS;
        homingStartMs = System.currentTimeMillis();
        freezeMode = false;
        turretIntegral = 0.0;
        lastDerivative = 0.0;
        lastAppliedPower = 0.0;
        lastTimeMs = System.currentTimeMillis();
    }

    public void setTargetPose(Pose target) {
        if (target != null) {
            this.targetPose = target;
        }
    }

    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;
        turretEncoderReference = turretMotor.getCurrentPosition();
    }

    public void resetPidState() {
        turretIntegral = 0.0;
        lastErrorTicks = 0;
        lastTimeMs = System.currentTimeMillis();
        lastAppliedPower = 0.0;
        lastDerivative = 0.0;
        manualActiveLast = false;
    }

    /**
     * Reset encoder to zero and recenter references
     */
    public void recenterAndResume(boolean resetEncoder) {
        homingMode = false;
        freezeMode = false;

        if (resetEncoder) {
            try {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception ignored) {}
        }

        captureReferences();
        resetPidState();
        lastTimeMs = System.currentTimeMillis();
    }

    public void update(boolean manualNow, double manualPower) {
        update(manualNow, manualPower, null);
    }

    /**
     * Main update loop
     */
    public void update(boolean manualNow, double manualPower, Pose robotPose) {
        long nowMs = System.currentTimeMillis();

        // ========== HOMING SWEEP OVERRIDES EVERYTHING ==========
        if (homingMode) {
            if (runHomingSweep()) {
                nowMs = System.currentTimeMillis();
            } else {
                return;
            }
        }

        // ========== FREEZE/HOLD MODE ==========
        if (freezeMode && !manualNow) {
            holdPositionTicks(freezeHoldTarget);
            return;
        }

        // Manual during freeze updates hold target
        if (freezeMode && manualNow) {
            applyManualPower(manualPower);
            freezeHoldTarget = turretMotor.getCurrentPosition();
            turretIntegral = 0.0;
            lastErrorTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            manualActiveLast = true;
            publishTelemetry();
            return;
        }

        // ========== MANUAL OVERRIDE ==========
        if (manualNow) {
            applyManualPower(manualPower);
            turretIntegral = 0.0;
            lastErrorTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            manualActiveLast = true;
            publishTelemetry();
            return;
        }

        // ========== TRANSITION FROM MANUAL TO AUTO ==========
        if (manualActiveLast && !manualNow) {
            captureReferences();
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        // ========== GET CURRENT HEADING ==========
        double currentHeadingRad = getHeadingRadians();

        // ========== CALCULATE TARGET ANGLE ==========
        double targetAngleRad;

        if (robotPose != null && targetPose != null) {
            // POSE-BASED TRACKING
            double dx = targetPose.getX() - robotPose.getX();
            double dy = targetPose.getY() - robotPose.getY();
            double fieldBearingToTarget = Math.atan2(dy, dx);

            double robotRelativeAngle = normalizeAngle(fieldBearingToTarget - robotPose.getHeading());
            targetAngleRad = robotRelativeAngle;

            lastBearingDeg = Math.toDegrees(fieldBearingToTarget);
            lastRobotRelativeAngleDeg = Math.toDegrees(robotRelativeAngle);

        } else {
            // FALLBACK: Heading-hold mode
            double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);
            targetAngleRad = -headingDelta;

            lastBearingDeg = 0.0;
            lastRobotRelativeAngleDeg = Math.toDegrees(targetAngleRad);
        }

        // ========== CALCULATE ANGULAR VELOCITY ==========
        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }
        lastHeadingRad = currentHeadingRad;

        // ========== CONVERT ANGLE TO ENCODER TICKS ==========
        double desiredTicksDouble = turretEncoderReference + targetAngleRad * TICKS_PER_RADIAN_SCALE;
        int desiredTicks = (int) Math.round(desiredTicksDouble);

        if (desiredTicks > TURRET_MAX_POS) desiredTicks = TURRET_MAX_POS;
        if (desiredTicks < TURRET_MIN_POS) desiredTicks = TURRET_MIN_POS;

        int currentTicks = turretMotor.getCurrentPosition();
        int errorTicks = desiredTicks - currentTicks;

        // ========== PID CONTROL ==========
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        if (Math.abs(errorTicks) > SMALL_DEADBAND_TICKS) {
            if (lastErrorTicks != 0 &&
                    ((errorTicks > 0 && lastErrorTicks < 0) || (errorTicks < 0 && lastErrorTicks > 0))) {
                turretIntegral *= 0.5;
            }
            turretIntegral += errorTicks * dt;
        } else {
            turretIntegral *= 0.90;
        }

        if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
        if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = DERIV_FILTER_ALPHA * rawDerivative +
                (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

        double pidOut = TURRET_KP * errorTicks +
                TURRET_KI * turretIntegral +
                TURRET_KD * derivativeFiltered;

        double ff = -angularVel * FF_GAIN;
        double cmdPower = pidOut + ff;

        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) {
            cmdPower = 0.0;
        }

        if (cmdPower > TURRET_MAX_POWER) cmdPower = TURRET_MAX_POWER;
        if (cmdPower < -TURRET_MAX_POWER) cmdPower = -TURRET_MAX_POWER;

        double applied = POWER_SMOOTH_ALPHA * lastAppliedPower +
                (1.0 - POWER_SMOOTH_ALPHA) * cmdPower;

        if ((currentTicks >= TURRET_MAX_POS && applied > 0.0) ||
                (currentTicks <= TURRET_MIN_POS && applied < 0.0)) {
            applied = 0.0;
        }

        turretMotor.setPower(applied);

        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;
        lastDesiredTicks = desiredTicks;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;

        publishTelemetry();
    }

    /**
     * Homing sweep state machine
     */
    private boolean runHomingSweep() {
        boolean resetNow = encoderResetTrigger != null && encoderResetTrigger.getAsBoolean();
        if (resetNow) {
            turretMotor.setPower(0.0);
            try {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception ignored) {}
            captureReferences();
            resetPidState();
            lastTimeMs = System.currentTimeMillis();
            lastAppliedPower = 0.0;
            lastDerivative = 0.0;

            // Enter freeze/hold after successful homing
            freezeMode = true;
            freezeHoldTarget = turretMotor.getCurrentPosition();
            homingMode = false;
            return true;
        }

        // Safety timeout
        if (System.currentTimeMillis() - homingStartMs > HOMING_TIMEOUT_MS) {
            turretMotor.setPower(0.0);
            homingMode = false;
            return true;
        }

        int current = turretMotor.getCurrentPosition();

        if (homingDirectionPos && current >= homingTarget - HOMING_TARGET_DEADBAND) {
            homingDirectionPos = false;
            homingTarget = -HOMING_AMPLITUDE_TICKS;
        } else if (!homingDirectionPos && current <= homingTarget + HOMING_TARGET_DEADBAND) {
            homingDirectionPos = true;
            homingTarget = HOMING_AMPLITUDE_TICKS;
        }

        double power = homingDirectionPos ? HOMING_POWER : -HOMING_POWER;

        if ((current >= TURRET_MAX_POS && power > 0.0) ||
                (current <= TURRET_MIN_POS && power < 0.0)) {
            power = 0.0;
        }

        turretMotor.setPower(power);

        lastDesiredTicks = homingTarget;
        lastErrorReported = homingTarget - current;
        lastPidOut = 0.0;
        lastFf = 0.0;

        publishTelemetry();
        return false;
    }

    /**
     * Hold fixed position (freeze mode)
     */
    private void holdPositionTicks(int targetTicks) {
        int currentTicks = turretMotor.getCurrentPosition();
        int errorTicks = targetTicks - currentTicks;

        long nowMs = System.currentTimeMillis();
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        if (Math.abs(errorTicks) > SMALL_DEADBAND_TICKS) {
            if (lastErrorTicks != 0 &&
                    ((errorTicks > 0 && lastErrorTicks < 0) || (errorTicks < 0 && lastErrorTicks > 0))) {
                turretIntegral *= 0.5;
            }
            turretIntegral += errorTicks * dt;
        } else {
            turretIntegral *= 0.90;
        }

        if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
        if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = DERIV_FILTER_ALPHA * rawDerivative +
                (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

        double pidOut = TURRET_KP * errorTicks + TURRET_KI * turretIntegral + TURRET_KD * derivativeFiltered;

        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) {
            pidOut = 0.0;
        }

        if (pidOut > TURRET_MAX_POWER) pidOut = TURRET_MAX_POWER;
        if (pidOut < -TURRET_MAX_POWER) pidOut = -TURRET_MAX_POWER;

        double applied = POWER_SMOOTH_ALPHA * lastAppliedPower + (1.0 - POWER_SMOOTH_ALPHA) * pidOut;

        if ((currentTicks >= TURRET_MAX_POS && applied > 0.0) ||
                (currentTicks <= TURRET_MIN_POS && applied < 0.0)) {
            applied = 0.0;
        }

        turretMotor.setPower(applied);

        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;
        lastDesiredTicks = targetTicks;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = 0.0;

        publishTelemetry();
    }

    private void applyManualPower(double manualPower) {
        int currentTicks = turretMotor.getCurrentPosition();

        double requested = manualPower;

        if ((currentTicks >= TURRET_MAX_POS && requested > 0.0) ||
                (currentTicks <= TURRET_MIN_POS && requested < 0.0)) {
            requested = 0.0;
        }

        if (requested > 1.0) requested = 1.0;
        if (requested < -1.0) requested = -1.0;

        turretMotor.setPower(requested);

        lastDesiredTicks = turretEncoderReference;
        lastErrorReported = lastDesiredTicks - currentTicks;
        lastPidOut = 0.0;
        lastFf = 0.0;
    }

    private double getHeadingRadians() {
        if (pinpoint != null) {
            try {
                return -pinpoint.getHeading(AngleUnit.RADIANS);
            } catch (Exception ignored) { }
        }

        if (imu == null) return 0.0;
        Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -o.firstAngle;
    }

    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    // Telemetry getters
    public int getLastDesiredTicks() { return lastDesiredTicks; }
    public int getLastErrorTicks() { return lastErrorReported; }
    public double getLastPidOut() { return lastPidOut; }
    public double getLastFf() { return lastFf; }
    public double getLastAppliedPower() { return lastAppliedPower; }
    public double getLastBearingDeg() { return lastBearingDeg; }
    public double getLastRobotRelativeAngleDeg() { return lastRobotRelativeAngleDeg; }
    public boolean isHomingMode() { return homingMode; }
    public boolean isFreezeMode() { return freezeMode; }

    private void publishTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("Turret.Desired", lastDesiredTicks);
        telemetry.addData("Turret.Current", turretMotor.getCurrentPosition());
        telemetry.addData("Turret.Error", lastErrorReported);
        telemetry.addData("Turret.FieldBearing", String.format("%.1f°", lastBearingDeg));
        telemetry.addData("Turret.RobotRelative", String.format("%.1f°", lastRobotRelativeAngleDeg));
        telemetry.addData("Turret.Power", String.format("%.3f", lastAppliedPower));
        telemetry.addData("Turret.Mode", freezeMode ? "FREEZE 🔒" : (homingMode ? "HOMING 🔄" : "TRACKING 🎯"));
    }
}