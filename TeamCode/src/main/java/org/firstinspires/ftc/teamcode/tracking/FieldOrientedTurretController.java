package org.firstinspires.ftc.teamcode.tracking;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.BooleanSupplier;

/**
 * FieldOrientedTurretController — Unified Pose-Based Turret Aiming
 *
 * Unlike TurretController (heading-hold only) or TurretGoalAimer (heading-hold + offset),
 * this controller computes the turret target from a SINGLE formula:
 *
 *   α              = atan2(goalY - robotY, goalX - robotX)   // absolute bearing to goal
 *   turretAngleRad = normalize(α - θ_capture)                // angle from turret-zero direction
 *   desiredTicks   = turretAngleRad * ticksPerRad            // convert to encoder ticks
 *
 * Where θ_capture is the robot heading at the moment captureReferences() was called
 * (i.e., the field-absolute direction that turret encoder = 0 corresponds to).
 *
 * This inherently handles BOTH rotation and translation because:
 *   - If the robot rotates: θ_capture doesn't change, but the PID sees the turret's
 *     real position drifting from the desired → corrects it.
 *     Wait — actually θ_capture is fixed at capture time. The turret encoder reads
 *     how far the turret has physically rotated from its zero. When the robot rotates,
 *     the turret motor doesn't move, so the encoder stays the same, but the desired
 *     ticks change because α changed (robot orientation changed → robot position hasn't,
 *     but heading used in α is field-absolute so α doesn't change with rotation alone).
 *     Actually α = atan2(goalY-robotY, goalX-robotX) does NOT depend on heading.
 *     So when the robot rotates in place, α stays constant, θ_capture is constant,
 *     so desiredTicks stays constant. But the turret is bolted to the chassis — when
 *     the chassis rotates, the turret physically rotates too, so the turret encoder
 *     stays the same while the turret now points in a different field direction.
 *
 *   KEY INSIGHT: The turret encoder measures turret angle RELATIVE TO THE CHASSIS.
 *   When the chassis rotates by Δθ, the turret's field-facing direction changes by Δθ
 *   even though the encoder doesn't move. So we need:
 *
 *     desiredTicks = normalize(α - θ_robot_now) * ticksPerRad
 *
 *   NOT θ_capture. Because we want the turret to be at angle (α - θ_robot_now) relative
 *   to the chassis to point at the goal in field coordinates.
 *
 *   But our encoder zero was set when θ_robot = θ_capture. So encoder 0 = chassis forward
 *   at θ_capture. If chassis has rotated to θ_now, then encoder 0 now points at θ_now
 *   in field coords. We want the turret to point at α. So:
 *
 *     desiredFieldAngle = α
 *     currentZeroFieldAngle = θ_now   (because encoder 0 = chassis forward = θ_now)
 *
 *   WRONG — encoder 0 doesn't equal chassis forward at θ_now. Encoder 0 was set when
 *   chassis was at θ_capture. If the chassis rotated by (θ_now - θ_capture), the turret
 *   encoder didn't change, but the field direction of encoder=0 changed by that amount.
 *
 *   Actually: the turret motor is on the chassis. When chassis rotates, turret rotates
 *   with it. The encoder measures turret-to-chassis angle. Encoder=0 always means
 *   "turret aligned with chassis forward". So:
 *     - Turret field direction = θ_robot + encoderAngle
 *     - We want turret field direction = α
 *     - So encoderAngle = α - θ_robot
 *     - desiredTicks = normalize(α - θ_robot) * ticksPerRad
 *
 *   This is INDEPENDENT of θ_capture entirely! The encoder offset just sets where
 *   virtual 0 is in raw encoder space. Since we captured when turret was at chassis
 *   forward, virtual 0 = chassis forward, and the formula above is correct.
 *
 * Feedforward: angular velocity of robot heading opposes the turret, so FF helps
 * the PID keep up with fast turns.
 *
 * Fallback: if no pose is available, falls back to pure heading-hold (same as
 * TurretController) so the turret still compensates for rotation.
 */
@Configurable
public class FieldOrientedTurretController {

    // Hardware
    private final DcMotor turretMotor;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;

    // Optional reset trigger (magnetic limit switch)
    private BooleanSupplier encoderResetTrigger = null;

    // Target field point to aim at (configurable — set to your goal/basket)
    @Sorter(sort = 0)
    public static double GOAL_X = 14.0;
    @Sorter(sort = 1)
    public static double GOAL_Y = 134.0;

    // Turret encoder hard limits
    @Sorter(sort = 2)
    public static int TURRET_MIN_POS = -1000;
    @Sorter(sort = 3)
    public static int TURRET_MAX_POS = 1000;
    @Sorter(sort = 4)
    public static double TICKS_PER_RADIAN_SCALE = 0.87;

    // PID config
    @Sorter(sort = 5)
    public static double TURRET_KP = 1.0;
    @Sorter(sort = 6)
    public static double TURRET_KI = 0.0;
    @Sorter(sort = 7)
    public static double TURRET_KD = 0.235;
    @Sorter(sort = 8)
    public static double TURRET_MAX_POWER = 1.0;

    // Feedforward and smoothing
    @Sorter(sort = 9)
    public static double FF_GAIN = 5.0;
    @Sorter(sort = 10)
    public static double POWER_SMOOTH_ALPHA = 0.9;
    @Sorter(sort = 11)
    public static double DERIV_FILTER_ALPHA = 1.0;

    // Deadband & anti-windup
    @Sorter(sort = 12)
    public static int SMALL_DEADBAND_TICKS = 3;
    @Sorter(sort = 13)
    public static double INTEGRAL_CLAMP = 50.0;

    // Rightward asymmetry damping
    @Sorter(sort = 14)
    public static double RIGHTWARD_ENCODER_DAMP = 0.9;
    @Sorter(sort = 15)
    public static int RIGHTWARD_DAMP_ERROR_WINDOW = 50;

    // Homing sweep configuration
    @Sorter(sort = 16)
    public static int HOMING_AMPLITUDE_TICKS = 300;
    @Sorter(sort = 17)
    public static double HOMING_POWER = 0.5;
    @Sorter(sort = 18)
    public static int HOMING_TARGET_DEADBAND = 12;
    @Sorter(sort = 19)
    public static long HOMING_TIMEOUT_MS = 3000;

    // Minimum distance to goal (inches) — below this, hold last valid target
    @Sorter(sort = 20)
    public static double MIN_GOAL_DISTANCE = 3.0;

    // ==================== Internal PID state ====================
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Heading reference (for fallback heading-hold when no pose available)
    private double headingReferenceRad = 0.0;
    private double lastHeadingRad = 0.0;
    private int turretEncoderReference = 0;

    // Encoder offset for virtual zero
    private int encoderOffset = 0;

    // Manual state
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

    // Last valid desired ticks (used when robot is too close to goal)
    private int lastValidDesiredTicks = 0;

    // ==================== Telemetry state ====================
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;
    private double lastHeadingDelta = 0.0;
    private double lastAngularVel = 0.0;
    private double lastBearingToGoalDeg = 0.0;
    private double lastDistanceToGoal = 0.0;
    private int lastDampedError = 0;

    // ==================== Constructors ====================

    public FieldOrientedTurretController(DcMotor turretMotor, GoBildaPinpointDriver pinpoint, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
        captureReferences();
        resetPidState();
    }

    public void setEncoderResetTrigger(BooleanSupplier trigger) {
        this.encoderResetTrigger = trigger;
    }

    public void setGoal(double x, double y) {
        GOAL_X = x;
        GOAL_Y = y;
    }

    // ==================== Virtual encoder ====================

    private int getVirtualEncoderPosition() {
        return turretMotor.getCurrentPosition() - encoderOffset;
    }

    /**
     * Capture current heading and turret encoder as reference.
     * Virtual encoder 0 = turret aligned with chassis forward at this moment.
     */
    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;
        encoderOffset = turretMotor.getCurrentPosition();
        turretEncoderReference = 0;
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

        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) pidOut = 0.0;
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
        lastBearingToGoalDeg = 0.0;
        lastDistanceToGoal = 0.0;
        lastDampedError = errorTicks;

        publishTelemetry();
    }

    // ==================== Main update ====================

    /**
     * Convenience overload — no pose available, pure heading-hold fallback.
     */
    public void update(boolean manualNow, double manualPower) {
        update(manualNow, manualPower, null);
    }

    /**
     * Main update loop — UNIFIED field-oriented aiming.
     *
     * When pose is available:
     *   α            = atan2(goalY - robotY, goalX - robotX)
     *   θ_robot      = robotPose.getHeading()
     *   turretAngle  = normalize(α - θ_robot)
     *   desiredTicks = turretAngle * ticksPerRad
     *
     *   This single formula accounts for BOTH rotation and translation.
     *   No heading-delta is needed because θ_robot is already in the formula.
     *
     * When pose is null (fallback):
     *   headingDelta = currentPinpointHeading - headingReferenceRad
     *   desiredTicks = -headingDelta * ticksPerRad
     *   (Same as original TurretController — pure heading-hold)
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

        double ticksPerRad = ((maxPosCfg - minPosCfg) / (2.0 * Math.PI)) * ticksPerRadScaleCfg;

        // --- Manual → auto transition: recapture references ---
        if (manualActiveLast && !manualNow) {
            captureReferences();
            currentVirtualTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        // ====================================================================
        // Read current heading from pinpoint (for FF and fallback)
        // ====================================================================
        double currentHeadingRad = getHeadingRadians();

        // Angular velocity (for feedforward — opposes yaw rate)
        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }
        lastHeadingRad = currentHeadingRad;

        // ====================================================================
        // Compute desired ticks
        // ====================================================================
        int desiredVirtualTicks;
        double headingDeltaForTelemetry = 0.0;

        if (robotPose != null) {
            // ── UNIFIED FIELD-ORIENTED AIMING ──
            double robotX = robotPose.getX();
            double robotY = robotPose.getY();
            double robotHeading = robotPose.getHeading(); // Pedro Pathing heading (radians, field frame)

            double dx = GOAL_X - robotX;
            double dy = GOAL_Y - robotY;
            double distToGoal = Math.sqrt(dx * dx + dy * dy);

            if (distToGoal >= MIN_GOAL_DISTANCE) {
                // α = absolute field bearing from robot to goal
                double alpha = Math.atan2(dy, dx);

                // Turret angle relative to chassis forward = α - θ_robot
                // Encoder 0 = chassis forward (set at captureReferences time, but
                // since we use the CURRENT robot heading, this works regardless
                // of when captureReferences was called).
                //
                // SIGN NOTE: We negate because positive encoder ticks = turret
                // rotates one way (CW/CCW depends on motor wiring). Adjust the
                // negate below if turret aims the wrong direction.
                double turretAngleRad = normalizeAngle(alpha - robotHeading);

                desiredVirtualTicks = (int) Math.round(turretAngleRad * ticksPerRad);
                lastValidDesiredTicks = desiredVirtualTicks;

                lastBearingToGoalDeg = Math.toDegrees(alpha);
                lastDistanceToGoal = distToGoal;
            } else {
                // Too close to goal — hold last valid target to avoid atan2 jitter
                desiredVirtualTicks = lastValidDesiredTicks;
                lastDistanceToGoal = distToGoal;
            }

            headingDeltaForTelemetry = normalizeAngle(robotHeading - headingReferenceRad);

        } else {
            // ── FALLBACK: pure heading-hold (no pose available) ──
            double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);
            headingDeltaForTelemetry = headingDelta;

            double desiredTicksDouble = turretEncoderReference - headingDelta * ticksPerRad;
            desiredVirtualTicks = (int) Math.round(desiredTicksDouble);
        }

        // Clamp to hard limits
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
        // PID + Feedforward
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

        // Feedforward: oppose robot yaw rate so PID doesn't have to do all the work
        double ff = -angularVel * ffGainCfg;
        double cmdPower = pidOut + ff;

        // Deadband — avoid tiny jittering
        if (Math.abs(errorTicks) <= deadbandCfg) {
            cmdPower = 0.0;
        }

        if (cmdPower > maxPowerCfg) cmdPower = maxPowerCfg;
        if (cmdPower < -maxPowerCfg) cmdPower = -maxPowerCfg;

        // Smooth applied power (exponential filter)
        double applied = powerSmoothCfg * lastAppliedPower + (1.0 - powerSmoothCfg) * cmdPower;

        // Hard-stop protection
        if ((currentVirtualTicks >= maxPosCfg && applied > 0.0) || (currentVirtualTicks <= minPosCfg && applied < 0.0)) {
            applied = 0.0;
        }

        turretMotor.setPower(applied);

        // ====================================================================
        // Update state
        // ====================================================================
        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;

        lastDesiredTicks = desiredBiased;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;
        lastHeadingDelta = headingDeltaForTelemetry;
        lastAngularVel = angularVel;
        lastDampedError = errorTicks;

        publishTelemetry();
    }

    // ==================== Homing sweep ====================

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
        lastBearingToGoalDeg = 0.0;
        lastDistanceToGoal = 0.0;
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
        lastBearingToGoalDeg = 0.0;
        lastDistanceToGoal = 0.0;
        lastDampedError = lastErrorReported;
    }

    // ==================== Heading source ====================

    private double getHeadingRadians() {
        if (pinpoint != null) {
            try {
                return -pinpoint.getHeading(AngleUnit.RADIANS);
            } catch (Exception ignored) {}
        }
        return 0.0;
    }

    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    // ==================== Telemetry getters ====================

    public int getLastDesiredTicks()     { return lastDesiredTicks; }
    public int getLastErrorTicks()       { return lastErrorReported; }
    public double getLastPidOut()        { return lastPidOut; }
    public double getLastFf()            { return lastFf; }
    public double getLastAppliedPower()  { return lastAppliedPower; }
    public int getEncoderOffset()        { return encoderOffset; }
    public int getVirtualPosition()      { return getVirtualEncoderPosition(); }
    public int getRawPosition()          { return turretMotor.getCurrentPosition(); }
    public double getLastHeadingDelta()  { return lastHeadingDelta; }
    public double getLastAngularVel()    { return lastAngularVel; }
    public double getLastBearingDeg()    { return lastBearingToGoalDeg; }
    public double getLastDistToGoal()    { return lastDistanceToGoal; }
    public int getLastDampedError()      { return lastDampedError; }
    public boolean isHomingMode()        { return homingMode; }
    public boolean isFreezeMode()        { return freezeMode; }

    private void publishTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("turret.desired", lastDesiredTicks);
        telemetry.addData("turret.virtualPos", getVirtualEncoderPosition());
        telemetry.addData("turret.rawPos", getRawPosition());
        telemetry.addData("turret.error", lastErrorReported);
        telemetry.addData("turret.dampedError", lastDampedError);
        telemetry.addData("turret.offset", encoderOffset);
        telemetry.addData("turret.bearing", String.format("%.1f°", lastBearingToGoalDeg));
        telemetry.addData("turret.distToGoal", String.format("%.1f in", lastDistanceToGoal));
        telemetry.addData("turret.freeze", freezeMode);
        telemetry.addData("turret.homing", homingMode);
        telemetry.addData("turret.mode", freezeMode ? "FREEZE" : (homingMode ? "HOMING" : "FIELD AIM"));
    }
}