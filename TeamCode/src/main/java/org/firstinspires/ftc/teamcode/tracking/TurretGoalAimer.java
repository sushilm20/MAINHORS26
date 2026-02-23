package org.firstinspires.ftc.teamcode.tracking;

import com.bylazar.configurables.annotations.Configurable;
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
 * TurretGoalAimer - Heading-Hold + Pose-Based Offset Tracking
 *
 * Architecture (matches TurretController + adds position offset):
 *   1. Heading-hold: compensate for robot yaw changes (identical to TurretController)
 *   2. Position offset: when a robot pose is available, compute how much the bearing
 *      to the goal has changed since the reference was captured (because the robot
 *      translated on the field), and add that as an offset in encoder ticks.
 *
 * If no pose is supplied, behaves identically to TurretController (pure heading-hold).
 */
@Configurable
public class TurretGoalAimer {

    // Hardware
    private final DcMotor turretMotor;
    private final BNO055IMU imu;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;

    // Optional reset trigger (magnetic limit switch)
    private BooleanSupplier encoderResetTrigger = null;

    // Target field point to aim at
    private Pose targetPose = new Pose(14, 134, 0);

    // Turret encoder hard limits (configurable)
    @Sorter(sort = 0)
    public static int TURRET_MIN_POS = -1000;
    @Sorter(sort = 1)
    public static int TURRET_MAX_POS = 1000;
    @Sorter(sort = 2)
    public static double TICKS_PER_RADIAN_SCALE = 0.87;

    // PID config
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

    // Rightward asymmetry control (carried over from TurretController)
    @Sorter(sort = 12)
    public static double RIGHTWARD_ENCODER_DAMP = 0.9;
    @Sorter(sort = 13)
    public static int RIGHTWARD_DAMP_ERROR_WINDOW = 50;

    // Homing sweep configuration
    @Sorter(sort = 14)
    public static int HOMING_AMPLITUDE_TICKS = 300;
    @Sorter(sort = 15)
    public static double HOMING_POWER = 0.5;
    @Sorter(sort = 16)
    public static int HOMING_TARGET_DEADBAND = 12;
    @Sorter(sort = 17)
    public static long HOMING_TIMEOUT_MS = 3000;

    // PID state
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Heading / encoder reference (same as TurretController)
    private double headingReferenceRad = 0.0;
    private double lastHeadingRad = 0.0;
    private int turretEncoderReference = 0;

    // Encoder offset for "virtual" zero after captureReferences (same as TurretController)
    private int encoderOffset = 0;

    // Reference bearing to goal at the moment captureReferences() was called.
    // This is the field-frame angle from the robot's reference position to the target.
    // As the robot translates, the current bearing changes, and the difference is the offset.
    private double referenceBearingRad = 0.0;

    // The robot pose at the time of captureReferences, used to compute referenceBearing.
    private Pose referencePose = null;

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
    private double lastHeadingDelta = 0.0;
    private double lastAngularVel = 0.0;
    private double lastPositionOffsetRad = 0.0;
    private int lastDampedError = 0;

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

    public void setEncoderResetTrigger(BooleanSupplier trigger) {
        this.encoderResetTrigger = trigger;
    }

    public void setTargetPose(Pose target) {
        if (target != null) {
            this.targetPose = target;
        }
    }

    // ==================== Virtual encoder (same as TurretController) ====================

    private int getVirtualEncoderPosition() {
        return turretMotor.getCurrentPosition() - encoderOffset;
    }

    /**
     * Capture heading + encoder reference AND snapshot the current bearing to the goal.
     * When robotPose is available, the bearing offset will be measured relative to this snapshot.
     */
    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;

        // Virtual encoder zero (same as TurretController)
        encoderOffset = turretMotor.getCurrentPosition();
        turretEncoderReference = 0;

        // Snapshot the bearing to goal from the current robot position.
        // This will be set properly when the first pose arrives; until then offset = 0.
        referencePose = null;
        referenceBearingRad = 0.0;
    }

    /**
     * Variant that also stores the robot pose for reference bearing calculation.
     */
    public void captureReferences(Pose robotPose) {
        captureReferences();
        if (robotPose != null && targetPose != null) {
            referencePose = robotPose;
            referenceBearingRad = Math.atan2(
                    targetPose.getY() - robotPose.getY(),
                    targetPose.getX() - robotPose.getX()
            );
        }
    }

    public void resetPidState() {
        turretIntegral = 0.0;
        lastErrorTicks = 0;
        lastTimeMs = System.currentTimeMillis();
        lastAppliedPower = 0.0;
        lastDerivative = 0.0;
        manualActiveLast = false;
    }

    // ==================== Homing ====================

    public void commandHomingSweep(boolean homingButtonPressed) {
        if (homingButtonPressed && !homingCommandPrev) {
            if (freezeMode) {
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

    public void recenterAndResume(boolean resetEncoder, Pose robotPose) {
        homingMode = false;
        freezeMode = false;

        if (resetEncoder) {
            try {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception ignored) {}
        }

        captureReferences(robotPose);
        resetPidState();
        lastTimeMs = System.currentTimeMillis();
    }

    public void disable() {
        freezeMode = false;
        turretIntegral = 0.0;
        lastErrorTicks = 0;
        lastDerivative = 0.0;
        lastAppliedPower = 0.0;
        lastTimeMs = -1L;
        manualActiveLast = false;
        turretMotor.setPower(0.0);
    }

    // ==================== Hold position (freeze mode PID) ====================

    public void holdPositionTicks(int targetVirtualTicks) {
        long nowMs = System.currentTimeMillis();

        if (targetVirtualTicks > TURRET_MAX_POS) targetVirtualTicks = TURRET_MAX_POS;
        if (targetVirtualTicks < TURRET_MIN_POS) targetVirtualTicks = TURRET_MIN_POS;

        int currentVirtualTicks = getVirtualEncoderPosition();
        int errorTicks = targetVirtualTicks - currentVirtualTicks;

        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        if (Math.abs(errorTicks) > SMALL_DEADBAND_TICKS) {
            if (lastErrorTicks != 0 && ((errorTicks > 0 && lastErrorTicks < 0) || (errorTicks < 0 && lastErrorTicks > 0))) {
                turretIntegral *= 0.5;
            }
            turretIntegral += errorTicks * dt;
        } else {
            turretIntegral *= 0.90;
        }

        if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
        if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = DERIV_FILTER_ALPHA * rawDerivative + (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

        double pidOut = TURRET_KP * errorTicks + TURRET_KI * turretIntegral + TURRET_KD * derivativeFiltered;

        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) {
            pidOut = 0.0;
        }

        if (pidOut > TURRET_MAX_POWER) pidOut = TURRET_MAX_POWER;
        if (pidOut < -TURRET_MAX_POWER) pidOut = -TURRET_MAX_POWER;

        double applied = POWER_SMOOTH_ALPHA * lastAppliedPower + (1.0 - POWER_SMOOTH_ALPHA) * pidOut;

        if ((currentVirtualTicks >= TURRET_MAX_POS && applied > 0.0) || (currentVirtualTicks <= TURRET_MIN_POS && applied < 0.0)) {
            applied = 0.0;
        }

        turretMotor.setPower(applied);

        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;
        lastDesiredTicks = targetVirtualTicks;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = 0.0;
        lastHeadingDelta = 0.0;
        lastAngularVel = 0.0;
        lastPositionOffsetRad = 0.0;
        lastDampedError = errorTicks;

        publishTelemetry();
    }

    // ==================== Main update ====================

    public void update(boolean manualNow, double manualPower) {
        update(manualNow, manualPower, null);
    }

    /**
     * Main update loop. Architecture:
     *
     * Step 1 (heading-hold, same as TurretController):
     *   headingDelta = currentHeading - referenceHeading
     *   desiredTicks = encoderRef - headingDelta * ticksPerRad
     *   → This alone keeps turret aimed at a fixed field direction.
     *
     * Step 2 (position offset, new):
     *   If robotPose is available, compute the field bearing from robot to goal NOW
     *   vs the field bearing from robot to goal at REFERENCE TIME.
     *   The difference (positionOffsetRad) represents how much the line-of-sight
     *   to the goal has rotated because the robot translated on the field.
     *   Add this offset in encoder ticks.
     *
     * Result: turret tracks the goal through both rotation AND translation.
     */
    public void update(boolean manualNow, double manualPower, Pose robotPose) {
        long nowMs = System.currentTimeMillis();

        // --- Homing sweep overrides everything ---
        if (homingMode) {
            if (runHomingSweep()) {
                nowMs = System.currentTimeMillis();
            } else {
                return;
            }
        }

        int currentVirtualTicks = getVirtualEncoderPosition();

        // --- Manual branch (always allowed, even when frozen) ---
        if (manualNow) {
            applyManualPower(manualPower, TURRET_MIN_POS, TURRET_MAX_POS, currentVirtualTicks);
            turretIntegral = 0.0;
            lastErrorTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            manualActiveLast = true;

            if (freezeMode) {
                freezeHoldTarget = getVirtualEncoderPosition();
            }
            publishTelemetry();
            return;
        }

        // --- Freeze/hold mode ---
        if (freezeMode) {
            holdPositionTicks(freezeHoldTarget);
            return;
        }

        // --- Config snapshot ---
        int minPosCfg = TURRET_MIN_POS;
        int maxPosCfg = TURRET_MAX_POS;
        double ticksPerRadScaleCfg = TICKS_PER_RADIAN_SCALE;
        double kpCfg = TURRET_KP;
        double kiCfg = TURRET_KI;
        double kdCfg = TURRET_KD;
        double maxPowerCfg = TURRET_MAX_POWER;
        double ffGainCfg = FF_GAIN;
        double powerSmoothCfg = POWER_SMOOTH_ALPHA;
        double derivFilterCfg = DERIV_FILTER_ALPHA;
        int deadbandCfg = SMALL_DEADBAND_TICKS;
        double integralClampCfg = INTEGRAL_CLAMP;
        double rightDampCfg = RIGHTWARD_ENCODER_DAMP;
        int rightDampWindowCfg = RIGHTWARD_DAMP_ERROR_WINDOW;

        // Derived ticks-per-radian (same formula as TurretController)
        double ticksPerRad = ((maxPosCfg - minPosCfg) / (2.0 * Math.PI)) * ticksPerRadScaleCfg;

        // --- Manual → auto transition ---
        if (manualActiveLast && !manualNow) {
            captureReferences(robotPose);
            currentVirtualTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        // --- Lazily capture reference bearing on first pose ---
        if (robotPose != null && referencePose == null && targetPose != null) {
            referencePose = robotPose;
            referenceBearingRad = Math.atan2(
                    targetPose.getY() - robotPose.getY(),
                    targetPose.getX() - robotPose.getX()
            );
        }

        // ====================================================================
        // STEP 1: Heading-hold (identical to TurretController)
        // ====================================================================
        double currentHeadingRad = getHeadingRadians();
        double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);

        // Angular velocity for feedforward
        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }
        lastHeadingRad = currentHeadingRad;

        // Base desired ticks from heading compensation (same as TurretController)
        double desiredTicksDouble = turretEncoderReference - headingDelta * ticksPerRad;

        // ====================================================================
        // STEP 2: Position offset (NEW — only when pose is available)
        // ====================================================================
        //
        // referenceBearingRad = atan2 from reference robot pos to goal (captured once)
        // currentBearingRad   = atan2 from current robot pos to goal (computed now)
        // positionOffsetRad   = difference = how much the line-of-sight rotated
        //                       because the robot moved sideways / forward / back
        //
        // This offset is in FIELD frame, so we add it directly to the heading-hold target.
        // The heading-hold already counters rotation; this adds the translation component.

        double positionOffsetRad = 0.0;

        if (robotPose != null && targetPose != null && referencePose != null) {
            double currentBearingRad = Math.atan2(
                    targetPose.getY() - robotPose.getY(),
                    targetPose.getX() - robotPose.getX()
            );
            // How much has the bearing to the goal changed since we captured references?
            positionOffsetRad = normalizeAngle(currentBearingRad - referenceBearingRad);
        }

        // Add position offset (converts field-frame bearing change to encoder ticks)
        desiredTicksDouble += positionOffsetRad * ticksPerRad;

        int desiredVirtualTicks = (int) Math.round(desiredTicksDouble);

        // Clamp
        if (desiredVirtualTicks > maxPosCfg) desiredVirtualTicks = maxPosCfg;
        if (desiredVirtualTicks < minPosCfg) desiredVirtualTicks = minPosCfg;

        // ====================================================================
        // Rightward asymmetry damping (same as TurretController)
        // ====================================================================
        int delta = desiredVirtualTicks - currentVirtualTicks;
        int desiredBiased = desiredVirtualTicks;
        boolean rightDampActive = delta > 0 && Math.abs(delta) <= rightDampWindowCfg;
        if (rightDampActive) {
            double scale = Math.max(0.0, 1.0 - rightDampCfg);
            desiredBiased = currentVirtualTicks + (int) Math.round(delta * scale);
        }

        if (desiredBiased > maxPosCfg) desiredBiased = maxPosCfg;
        if (desiredBiased < minPosCfg) desiredBiased = minPosCfg;

        int errorTicks = desiredBiased - currentVirtualTicks;

        // ====================================================================
        // PID + FF (identical to TurretController)
        // ====================================================================
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        // Integral gating
        if (Math.abs(errorTicks) > deadbandCfg) {
            if (lastErrorTicks != 0 && ((errorTicks > 0 && lastErrorTicks < 0) || (errorTicks < 0 && lastErrorTicks > 0))) {
                turretIntegral *= 0.5;
            }
            turretIntegral += errorTicks * dt;
        } else {
            turretIntegral *= 0.90;
        }

        if (turretIntegral > integralClampCfg) turretIntegral = integralClampCfg;
        if (turretIntegral < -integralClampCfg) turretIntegral = -integralClampCfg;

        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = derivFilterCfg * rawDerivative + (1.0 - derivFilterCfg) * lastDerivative;

        double pidOut = kpCfg * errorTicks + kiCfg * turretIntegral + kdCfg * derivativeFiltered;

        double ff = -angularVel * ffGainCfg;
        double cmdPower = pidOut + ff;

        if (Math.abs(errorTicks) <= deadbandCfg) {
            cmdPower = 0.0;
        }

        if (cmdPower > maxPowerCfg) cmdPower = maxPowerCfg;
        if (cmdPower < -maxPowerCfg) cmdPower = -maxPowerCfg;

        double applied = powerSmoothCfg * lastAppliedPower + (1.0 - powerSmoothCfg) * cmdPower;

        if ((currentVirtualTicks >= maxPosCfg && applied > 0.0) || (currentVirtualTicks <= minPosCfg && applied < 0.0)) {
            applied = 0.0;
        }

        turretMotor.setPower(applied);

        // Update state
        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;

        lastDesiredTicks = desiredBiased;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;
        lastHeadingDelta = headingDelta;
        lastAngularVel = angularVel;
        lastPositionOffsetRad = positionOffsetRad;
        lastDampedError = errorTicks;

        publishTelemetry();
    }

    // ==================== Homing sweep (same as TurretController) ====================

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

            freezeMode = true;
            freezeHoldTarget = getVirtualEncoderPosition();
            homingMode = false;
            return true;
        }

        if (System.currentTimeMillis() - homingStartMs > HOMING_TIMEOUT_MS) {
            turretMotor.setPower(0.0);
            homingMode = false;
            return true;
        }

        int current = getVirtualEncoderPosition();

        if (homingDirectionPos && current >= homingTarget - HOMING_TARGET_DEADBAND) {
            homingDirectionPos = false;
            homingTarget = -HOMING_AMPLITUDE_TICKS;
        } else if (!homingDirectionPos && current <= homingTarget + HOMING_TARGET_DEADBAND) {
            homingDirectionPos = true;
            homingTarget = HOMING_AMPLITUDE_TICKS;
        }

        double power = homingDirectionPos ? HOMING_POWER : -HOMING_POWER;

        if ((current >= TURRET_MAX_POS && power > 0.0) || (current <= TURRET_MIN_POS && power < 0.0)) {
            power = 0.0;
        }

        turretMotor.setPower(power);

        lastDesiredTicks = homingTarget;
        lastErrorReported = homingTarget - current;
        lastPidOut = 0.0;
        lastFf = 0.0;
        lastHeadingDelta = 0.0;
        lastAngularVel = 0.0;
        lastPositionOffsetRad = 0.0;
        lastDampedError = lastErrorReported;

        publishTelemetry();
        return false;
    }

    // ==================== Manual ====================

    private void applyManualPower(double manualPower, int minPosCfg, int maxPosCfg, int currentVirtualTicks) {
        double requested = manualPower;
        if ((currentVirtualTicks >= maxPosCfg && requested > 0.0) || (currentVirtualTicks <= minPosCfg && requested < 0.0)) {
            requested = 0.0;
        }

        if (requested > 1.0) requested = 1.0;
        if (requested < -1.0) requested = -1.0;

        turretMotor.setPower(requested);
        lastDesiredTicks = turretEncoderReference;
        lastErrorReported = lastDesiredTicks - currentVirtualTicks;
        lastPidOut = 0.0;
        lastFf = 0.0;
        lastHeadingDelta = 0.0;
        lastAngularVel = 0.0;
        lastPositionOffsetRad = 0.0;
        lastDampedError = lastErrorReported;
    }

    // ==================== Heading source ====================

    private double getHeadingRadians() {
        if (pinpoint != null) {
            try {
                return -pinpoint.getHeading(AngleUnit.RADIANS);
            } catch (Exception ignored) {}
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

    // ==================== Telemetry getters ====================

    public int getLastDesiredTicks() { return lastDesiredTicks; }
    public int getLastErrorTicks() { return lastErrorReported; }
    public double getLastPidOut() { return lastPidOut; }
    public double getLastFf() { return lastFf; }
    public double getLastAppliedPower() { return lastAppliedPower; }
    public int getEncoderOffset() { return encoderOffset; }
    public int getVirtualPosition() { return getVirtualEncoderPosition(); }
    public int getRawPosition() { return turretMotor.getCurrentPosition(); }
    public double getLastHeadingDelta() { return lastHeadingDelta; }
    public double getLastAngularVel() { return lastAngularVel; }
    public double getLastPositionOffsetRad() { return lastPositionOffsetRad; }
    public double getLastPositionOffsetDeg() { return Math.toDegrees(lastPositionOffsetRad); }
    public int getLastDampedError() { return lastDampedError; }
    public boolean isHomingMode() { return homingMode; }
    public boolean isFreezeMode() { return freezeMode; }

    private void publishTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("turret.desired", lastDesiredTicks);
        telemetry.addData("turret.virtualPos", getVirtualEncoderPosition());
        telemetry.addData("turret.rawPos", getRawPosition());
        telemetry.addData("turret.error", lastErrorReported);
        telemetry.addData("turret.dampedError", lastDampedError);
        telemetry.addData("turret.offset", encoderOffset);
        telemetry.addData("turret.headingDelta", String.format("%.2f°", Math.toDegrees(lastHeadingDelta)));
        telemetry.addData("turret.posOffset", String.format("%.2f°", Math.toDegrees(lastPositionOffsetRad)));
        telemetry.addData("turret.freeze", freezeMode);
        telemetry.addData("turret.homing", homingMode);
        telemetry.addData("turret.mode", freezeMode ? "FREEZE" : (homingMode ? "HOMING" : "TRACKING"));
    }
}