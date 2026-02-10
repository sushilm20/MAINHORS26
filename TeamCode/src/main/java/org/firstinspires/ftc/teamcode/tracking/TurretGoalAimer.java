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

/**
 * TurretGoalAimer - Simplified Pose-Based Point Tracking
 *
 * Aims turret at a fixed field point using:
 * - Robot pose (X, Y, heading) from GoBilda Pinpoint odometry
 * - Simple geometric bearing calculation
 * - PID control with feedforward
 *
 * Algorithm:
 * 1. Calculate field-relative bearing to target: atan2(dy, dx)
 * 2. Convert to robot-relative angle: bearing - robotHeading
 * 3. Convert to turret encoder ticks
 * 4. Apply PID + feedforward control
 */
public class TurretGoalAimer {

    // Hardware
    private final DcMotor turretMotor;
    private final BNO055IMU imu;                     // fallback
    private final GoBildaPinpointDriver pinpoint;    // preferred
    private final Telemetry telemetry;

    // Target (default to blue goal)
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

    // PID state
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Reference state (for heading-hold fallback)
    private double headingReferenceRad = 0.0;
    private double lastHeadingRad = 0.0;
    private int turretEncoderReference = 0;
    private boolean manualActiveLast = false;

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

    // Backward-compatible constructor (no Pinpoint)
    public TurretGoalAimer(DcMotor turretMotor, BNO055IMU imu, Telemetry telemetry) {
        this(turretMotor, imu, null, telemetry);
    }

    /** Set the target field position to aim at */
    public void setTargetPose(Pose target) {
        if (target != null) {
            this.targetPose = target;
        }
    }

    /** Capture current heading and turret encoder as reference */
    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;
        turretEncoderReference = turretMotor.getCurrentPosition();
    }

    /** Reset PID internal state */
    public void resetPidState() {
        turretIntegral = 0.0;
        lastErrorTicks = 0;
        lastTimeMs = System.currentTimeMillis();
        lastAppliedPower = 0.0;
        lastDerivative = 0.0;
        manualActiveLast = false;
    }

    /** Manual-only update (no pose) */
    public void update(boolean manualNow, double manualPower) {
        update(manualNow, manualPower, null);
    }

    /**
     * Main update loop - call this every control cycle
     *
     * @param manualNow true if driver is manually controlling turret
     * @param manualPower manual power (-1.0 to 1.0)
     * @param robotPose current robot pose from Follower.getPose() (can be null)
     */
    public void update(boolean manualNow, double manualPower, Pose robotPose) {
        long nowMs = System.currentTimeMillis();

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
            // POSE-BASED TRACKING: Calculate bearing from robot to target

            // 1. Calculate field-relative bearing to target
            double dx = targetPose.getX() - robotPose.getX();
            double dy = targetPose.getY() - robotPose.getY();
            double fieldBearingToTarget = Math.atan2(dy, dx);  // radians, field coordinates

            // 2. Convert to robot-relative angle
            // The turret is mounted on the robot, so we need angle relative to robot's heading
            double robotRelativeAngle = normalizeAngle(fieldBearingToTarget - robotPose.getHeading());

            // 3. This is the angle the turret needs to be at (relative to its reference)
            targetAngleRad = robotRelativeAngle;

            // Save for telemetry
            lastBearingDeg = Math.toDegrees(fieldBearingToTarget);
            lastRobotRelativeAngleDeg = Math.toDegrees(robotRelativeAngle);

        } else {
            // FALLBACK: Heading-hold mode (like TurretController)
            // If no pose data, just compensate for heading changes
            double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);
            targetAngleRad = -headingDelta;  // negative to counter-rotate

            lastBearingDeg = 0.0;
            lastRobotRelativeAngleDeg = Math.toDegrees(targetAngleRad);
        }

        // ========== CALCULATE ANGULAR VELOCITY (for feedforward) ==========
        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;  // rad/s
        }
        lastHeadingRad = currentHeadingRad;

        // ========== CONVERT ANGLE TO ENCODER TICKS ==========
        // The turret encoder reference is where we captured it
        // We want the turret to rotate by targetAngleRad from that reference
        double desiredTicksDouble = turretEncoderReference + targetAngleRad * TICKS_PER_RADIAN_SCALE;
        int desiredTicks = (int) Math.round(desiredTicksDouble);

        // Clamp to limits
        if (desiredTicks > TURRET_MAX_POS) desiredTicks = TURRET_MAX_POS;
        if (desiredTicks < TURRET_MIN_POS) desiredTicks = TURRET_MIN_POS;

        // ========== CALCULATE ERROR ==========
        int currentTicks = turretMotor.getCurrentPosition();
        int errorTicks = desiredTicks - currentTicks;

        // ========== PID CONTROL ==========
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        // Integral (with sign-change damping and clamping)
        if (Math.abs(errorTicks) > SMALL_DEADBAND_TICKS) {
            // Dampen integral on sign change (reduce overshoot)
            if (lastErrorTicks != 0 &&
                    ((errorTicks > 0 && lastErrorTicks < 0) || (errorTicks < 0 && lastErrorTicks > 0))) {
                turretIntegral *= 0.5;
            }
            turretIntegral += errorTicks * dt;
        } else {
            turretIntegral *= 0.90;  // decay when near target
        }

        if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
        if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

        // Derivative (filtered to reduce noise)
        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = DERIV_FILTER_ALPHA * rawDerivative +
                (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

        // PID output
        double pidOut = TURRET_KP * errorTicks +
                TURRET_KI * turretIntegral +
                TURRET_KD * derivativeFiltered;

        // Feedforward: oppose robot rotation to maintain lock
        double ff = -angularVel * FF_GAIN;

        // ========== COMMAND POWER ==========
        double cmdPower = pidOut + ff;

        // Deadband (avoid jitter near target)
        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) {
            cmdPower = 0.0;
        }

        // Clamp to max power
        if (cmdPower > TURRET_MAX_POWER) cmdPower = TURRET_MAX_POWER;
        if (cmdPower < -TURRET_MAX_POWER) cmdPower = -TURRET_MAX_POWER;

        // Smooth power changes
        double applied = POWER_SMOOTH_ALPHA * lastAppliedPower +
                (1.0 - POWER_SMOOTH_ALPHA) * cmdPower;

        // Stop at hard limits
        if ((currentTicks >= TURRET_MAX_POS && applied > 0.0) ||
                (currentTicks <= TURRET_MIN_POS && applied < 0.0)) {
            applied = 0.0;
        }

        // ========== APPLY TO MOTOR ==========
        turretMotor.setPower(applied);

        // ========== UPDATE STATE ==========
        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;
        lastDesiredTicks = desiredTicks;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;

        publishTelemetry();
    }

    private void applyManualPower(double manualPower) {
        int currentTicks = turretMotor.getCurrentPosition();

        double requested = manualPower;

        // Stop at limits
        if ((currentTicks >= TURRET_MAX_POS && requested > 0.0) ||
                (currentTicks <= TURRET_MIN_POS && requested < 0.0)) {
            requested = 0.0;
        }

        // Clamp
        if (requested > 1.0) requested = 1.0;
        if (requested < -1.0) requested = -1.0;

        turretMotor.setPower(requested);

        lastDesiredTicks = turretEncoderReference;
        lastErrorReported = lastDesiredTicks - currentTicks;
        lastPidOut = 0.0;
        lastFf = 0.0;
    }

    private double getHeadingRadians() {
        // Prefer Pinpoint; invert to match BNO055 sign convention
        if (pinpoint != null) {
            try {
                return -pinpoint.getHeading(AngleUnit.RADIANS);
            } catch (Exception ignored) { }
        }

        // Fallback to BNO055 IMU
        if (imu == null) return 0.0;
        Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -o.firstAngle;
    }

    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    // ========== TELEMETRY GETTERS ==========
    public int getLastDesiredTicks() { return lastDesiredTicks; }
    public int getLastErrorTicks() { return lastErrorReported; }
    public double getLastPidOut() { return lastPidOut; }
    public double getLastFf() { return lastFf; }
    public double getLastAppliedPower() { return lastAppliedPower; }
    public double getLastBearingDeg() { return lastBearingDeg; }
    public double getLastRobotRelativeAngleDeg() { return lastRobotRelativeAngleDeg; }

    private void publishTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("Turret.Desired", lastDesiredTicks);
        telemetry.addData("Turret.Current", turretMotor.getCurrentPosition());
        telemetry.addData("Turret.Error", lastErrorReported);
        telemetry.addData("Turret.FieldBearing", String.format("%.1f°", lastBearingDeg));
        telemetry.addData("Turret.RobotRelative", String.format("%.1f°", lastRobotRelativeAngleDeg));
        telemetry.addData("Turret.Power", String.format("%.3f", lastAppliedPower));
    }
}