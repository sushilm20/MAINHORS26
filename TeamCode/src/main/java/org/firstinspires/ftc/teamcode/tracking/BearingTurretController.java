package org.firstinspires.ftc.teamcode.tracking;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

/**
 * BearingTurretController — Pure Pose-Based Turret Aiming
 *
 * Uses the Pedro Pathing Follower pose as the SOLE source of truth.
 * No IMU, no heading-hold layer, no reference-delta system.
 *
 * EVERY LOOP:
 *   1. Get robot pose from Follower: (x, y, heading)
 *   2. Compute field bearing from robot → goal:
 *        bearingRad = atan2(goalY - robotY, goalX - robotX)
 *   3. Compute turret-relative angle:
 *        turretAngleRad = normalize(bearingRad - robotHeading)
 *   4. Convert to encoder ticks:
 *        desiredTicks = turretAngleRad × (TICKS_PER_FULL_ROTATION / 2π) × ENCODER_SIGN
 *   5. PID on the error (desired - current), enforce encoder limits, apply power.
 *
 * SETUP:
 *   1. Zero the turret encoder when the turret is pointing straight forward on the robot.
 *      (0 ticks = turret aligned with chassis forward direction)
 *   2. Set the goal position (field coordinates, inches).
 *   3. Provide the Follower reference (call setFollower or pass to constructor).
 *   4. Call update() each loop AFTER follower.update().
 *
 * MANUAL OFFSET:
 *   When the driver uses manual control (bumpers/stick), the controller tracks how
 *   far they moved the turret. On release, that movement is stored as an angular
 *   offset (manualOffsetRad) that persists on top of the bearing calculation.
 *   Call clearManualOffset() to snap back to pure goal tracking.
 *
 * BEARING MATH (from StackExchange):
 *   Standard atan2 bearing:  θ = atan2(Δy, Δx)
 *   This matches Pedro Pathing's heading convention (0 = +X east, CCW positive).
 */
@Configurable
public class BearingTurretController {

    // ── Hardware ──
    private final DcMotor turretMotor;
    private final Telemetry telemetry;  // may be null

    // ── Pose source ──
    private Follower follower;

    // ── Encoder reset trigger (e.g., magnetic limit switch) ──
    private BooleanSupplier encoderResetTrigger = null;

    // ══════════════════════════════════════════════════
    //  CONFIGURABLE CONSTANTS
    // ══════════════════════════════════════════════════

    // Goal position on the field (inches, Pedro Pathing coordinates)
    @Sorter(sort = 0) public static double GOAL_X = 13.0;
    @Sorter(sort = 1) public static double GOAL_Y = 135.0;

    // Encoder geometry
    // Total encoder ticks for one full 360° turret rotation
    @Sorter(sort = 2) public static double TICKS_PER_FULL_ROTATION = 1660;
    // +1 if positive ticks = CCW (left), -1 if positive ticks = CW (right)
    // If turret overturns on rotation, flip this sign.
    @Sorter(sort = 3) public static double ENCODER_SIGN = - 1.0; //keep liek this to have correct adjustment direction

    // Encoder hard limits (raw ticks)
    @Sorter(sort = 4) public static int TURRET_MIN_TICKS = -830;
    @Sorter(sort = 5) public static int TURRET_MAX_TICKS = 830;

    // PID gains (error is in degrees)
    @Sorter(sort = 6)  public static double KP = 0.016;
    @Sorter(sort = 7)  public static double KI = 0.0;
    @Sorter(sort = 8)  public static double KD = 0.003;
    @Sorter(sort = 9)  public static double MAX_POWER = 1.0;

    // Deadband: if error < this many degrees, output 0 (prevents jitter)
    @Sorter(sort = 10) public static double DEADBAND_DEG = 1.5;

    // Integral anti-windup clamp (in degree·seconds)
    @Sorter(sort = 11) public static double INTEGRAL_CLAMP_DEG = 100.0;

    // Power smoothing (EMA alpha: 0 = no smoothing, 1 = never changes)
    @Sorter(sort = 12) public static double POWER_SMOOTH = 0.67;

    // Minimum distance to goal (inches). If closer than this, hold last known aim.
    // Prevents wild oscillation when robot is nearly on top of the goal.
    @Sorter(sort = 13) public static double MIN_GOAL_DIST = 3.0;

    // Turret forward offset in radians: if the turret's "zero" doesn't point
    // exactly along the robot's +X axis, add an offset here.
    @Sorter(sort = 14) public static double TURRET_FORWARD_OFFSET_RAD = Math.toRadians(15);

    // Homing sweep configuration
    @Sorter(sort = 15) public static int HOMING_AMP_TICKS = 300;
    @Sorter(sort = 16) public static double HOMING_POWER = 0.5;
    @Sorter(sort = 17) public static int HOMING_DEADBAND = 12;
    @Sorter(sort = 18) public static long HOMING_TIMEOUT_MS = 3000;

    // ── Velocity compensation ──
    // Feedforward gain: how many degrees of turret lead per degree/sec of bearing rate.
    // Higher = more aggressive lead. Start at ~0.05, tune up.
    @Sorter(sort = 19) public static double VELOCITY_COMP_GAIN = 0.25;

    // Low-pass filter alpha for bearing rate (0 = no smoothing, 1 = infinite smoothing)
    @Sorter(sort = 20) public static double BEARING_RATE_FILTER = 0.6;

    // Rightward asymmetry damping (bias desired target toward current)
    // Same behavior concept as TurretController:
    // - applies only when desired is to the "rightward" side (delta > 0 in tick space)
    // - only when within a small error window
    @Sorter(sort = 21) public static double RIGHTWARD_ENCODER_DAMP = 0.70;
    @Sorter(sort = 22) public static int RIGHTWARD_DAMP_ERROR_WINDOW = 100;

    // ══════════════════════════════════════════════════
    //  INTERNAL STATE
    // ══════════════════════════════════════════════════

    // PID state
    private double integral = 0.0;
    private double lastErrorDeg = 0.0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private boolean manualWasActive = false;

    // Last valid aim direction (used when too close to goal)
    private double lastValidDesiredRad = 0.0;

    // Manual aim offset: when the driver nudges the turret and releases,
    // the angular displacement is stored here and added to the bearing calc.
    private double manualOffsetRad = 0.0;
    private int manualStartTicks = 0;

    // Homing state
    private boolean homingMode = false;
    private boolean homingBtnPrev = false;
    private boolean homingDirPos = true;
    private int homingTarget = HOMING_AMP_TICKS;
    private long homingStartMs = 0L;

    // Freeze/hold state (after homing finds the limit switch)
    private boolean freezeMode = false;
    public int freezeTargetTicks = 0;

    // Tracks whether the controller has received at least one valid pose.
    // Prevents the turret from lurching on the very first loop cycle
    // before the Follower localizer has settled.
    private boolean hasFirstPose = false;

    // Telemetry readouts
    private double tBearingDeg, tTurretDesiredDeg, tErrorDeg;
    private double tRobotX, tRobotY, tRobotHeadingDeg;
    private double tDistToGoal, tAppliedPower;
    private int tEncoderTicks, tDesiredTicks;

    // Velocity compensation state
    private double lastBearingRad = Double.NaN;
    private double filteredBearingRateDegPerSec = 0.0;

    // Velocity compensation telemetry
    private double tVelCompDeg;

    // ══════════════════════════════════════════════════
    //  CONSTRUCTORS
    // ══════════════════════════════════════════════════

    /**
     * Full constructor.
     *
     * @param turretMotor the turret DcMotor (must already be configured: direction, mode)
     * @param follower    Pedro Pathing Follower (provides robot pose)
     * @param telemetry   optional FTC Telemetry for debugging (may be null)
     */
    public BearingTurretController(DcMotor turretMotor, Follower follower, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.follower = follower;
        this.telemetry = telemetry;
    }

    /**
     * Constructor without telemetry.
     */
    public BearingTurretController(DcMotor turretMotor, Follower follower) {
        this(turretMotor, follower, null);
    }

    // ══════════════════════════════════════════════════
    //  SETUP METHODS
    // ══════════════════════════════════════════════════

    /** Set/change the Follower reference (e.g., if created after this controller). */
    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    /** Set the goal the turret should aim at (field inches). */
    public void setGoal(double x, double y) {
        GOAL_X = x;
        GOAL_Y = y;
    }

    /** Provide a limit-switch trigger for homing. */
    public void setEncoderResetTrigger(BooleanSupplier trigger) {
        this.encoderResetTrigger = trigger;
    }

    /** Zero the turret encoder. Call this when turret is physically pointing forward. */
    public void zeroEncoder() {
        try {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception ignored) {}
        clearPid();
    }

    /** Full reset: exit homing/freeze, zero encoder, clear manual offset. */
    public void fullReset() {
        homingMode = false;
        freezeMode = false;
        manualOffsetRad = 0.0;
        hasFirstPose = false;
        zeroEncoder();
    }

    /** Clear PID state (useful on mode transitions). */
    public void clearPid() {
        integral = 0.0;
        lastErrorDeg = 0.0;
        lastTimeMs = System.currentTimeMillis();
        lastAppliedPower = 0.0;
        manualWasActive = false;
        lastBearingRad = Double.NaN;
        filteredBearingRateDegPerSec = 0.0;
    }

    /** Stop the turret motor and reset all state. */
    public void disable() {
        homingMode = false;
        freezeMode = false;
        integral = 0.0;
        lastErrorDeg = 0.0;
        lastAppliedPower = 0.0;
        lastTimeMs = -1L;
        lastBearingRad = Double.NaN;
        filteredBearingRateDegPerSec = 0.0;
        turretMotor.setPower(0.0);
    }

    /**
     * Reset the manual offset back to zero (turret aims purely at the goal).
     * Call this on a button press to "re-center" the aim.
     */
    public void clearManualOffset() {
        manualOffsetRad = 0.0;
    }

    /** Get the current manual offset in degrees (useful for telemetry). */
    public double getManualOffsetDeg() {
        return Math.toDegrees(manualOffsetRad);
    }

    /** Whether the controller is currently in freeze/hold mode. */
    public boolean isFreezeMode() {
        return freezeMode;
    }

    /** Whether the controller is currently running a homing sweep. */
    public boolean isHomingMode() {
        return homingMode;
    }

    /**
     * Enter freeze mode: hold current encoder position, stop tracking.
     */
    public void freeze() {
        freezeMode = true;
        freezeTargetTicks = turretMotor.getCurrentPosition();
        clearPid();
    }

    /**
     * Exit freeze mode and resume tracking.
     */
    public void unfreeze() {
        freezeMode = false;
        clearPid();
    }

    // ══════════════════════════════════════════════════
    //  CORE MATH
    // ══════════════════════════════════════════════════

    /**
     * Bearing from point A to point B in radians.
     * Uses standard atan2 convention: 0 = +X (east), CCW positive.
     * This matches Pedro Pathing's heading convention.
     */
    private static double bearingRad(double ax, double ay, double bx, double by) {
        return Math.atan2(by - ay, bx - ax); //w arc tan
    }

    /**
     * Normalize an angle to [-π, +π].
     */
    private static double normalizeAngle(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    /**
     * Convert a turret-relative angle (radians) to encoder ticks.
     *
     * turretAngleRad = 0 means turret points forward.
     * ENCODER_SIGN maps the math-convention angle to the motor's tick direction.
     */
    private double radiansToTicks(double turretAngleRad) {
        double ticksPerRad = TICKS_PER_FULL_ROTATION / (2.0 * Math.PI);
        return turretAngleRad * ticksPerRad * ENCODER_SIGN;
    }

    /**
     * Convert encoder ticks to turret angle in radians.
     */
    private double ticksToRadians(int ticks) {
        double ticksPerRad = TICKS_PER_FULL_ROTATION / (2.0 * Math.PI);
        return ticks / (ticksPerRad * ENCODER_SIGN);
    }

    // ══════════════════════════════════════════════════
    //  HOMING SWEEP
    // ══════════════════════════════════════════════════

    /**
     * Command a homing sweep via button. Rising edge starts sweep;
     * if frozen, rising edge unfreezes and resumes tracking.
     */
    public void commandHomingSweep(boolean buttonPressed) {
        if (buttonPressed && !homingBtnPrev) {
            if (freezeMode) {
                freezeMode = false;
                clearPid();
            } else {
                homingMode = true;
                homingDirPos = true;
                homingTarget = HOMING_AMP_TICKS;
                homingStartMs = System.currentTimeMillis();
                freezeMode = false;
                lastAppliedPower = 0.0;
            }
        }
        homingBtnPrev = buttonPressed;
    }

    /**
     * Run the homing sweep state machine.
     * Returns true if homing finished this cycle.
     */
    private boolean runHomingSweep() {
        // Immediate stop if limit switch trips
        if (encoderResetTrigger != null && encoderResetTrigger.getAsBoolean()) {
            turretMotor.setPower(0.0);
            zeroEncoder();
            freezeMode = true;
            freezeTargetTicks = 0;
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

        // Oscillate between -amp and +amp
        if (homingDirPos && current >= homingTarget - HOMING_DEADBAND) {
            homingDirPos = false;
            homingTarget = -HOMING_AMP_TICKS;
        } else if (!homingDirPos && current <= homingTarget + HOMING_DEADBAND) {
            homingDirPos = true;
            homingTarget = HOMING_AMP_TICKS;
        }

        double power = homingDirPos ? HOMING_POWER : -HOMING_POWER;

        // Respect encoder hard limits
        if ((current >= TURRET_MAX_TICKS && power > 0) ||
                (current <= TURRET_MIN_TICKS && power < 0)) {
            power = 0.0;
        }

        turretMotor.setPower(power);
        return false;
    }

    // ══════════════════════════════════════════════════
    //  FREEZE HOLD (simple P on raw ticks)
    // ══════════════════════════════════════════════════

    public void holdRawTicks(int targetTicks) {
        int current = turretMotor.getCurrentPosition();
        int err = targetTicks - current;

        double power = 0.0;
        if (Math.abs(err) > 3) {
            double ticksPerDeg = TICKS_PER_FULL_ROTATION / 360.0;
            double errDeg = err / ticksPerDeg;
            power = KP * errDeg;
            if (power > MAX_POWER) power = MAX_POWER;
            if (power < -MAX_POWER) power = -MAX_POWER;
        }

        if ((current >= TURRET_MAX_TICKS && power > 0) ||
                (current <= TURRET_MIN_TICKS && power < 0)) {
            power = 0.0;
        }

        turretMotor.setPower(power);
    }

    // ══════════════════════════════════════════════════
    //  MAIN UPDATE — Call this every loop AFTER follower.update()
    // ══════════════════════════════════════════════════

    /**
     * Main update. Call from your OpMode loop().
     *
     * @param manualNow   true if the driver is manually controlling the turret this cycle
     * @param manualPower the manual power (-1 to +1) from the joystick
     */
    public void update(boolean manualNow, double manualPower) {
        long nowMs = System.currentTimeMillis();

        // ── Homing overrides everything ──
        if (homingMode) {
            if (runHomingSweep()) {
                nowMs = System.currentTimeMillis();
                // homing finished, fall through
            } else {
                publishTelemetry();
                return; // still homing
            }
        }

        int rawTicks = turretMotor.getCurrentPosition();

        // ── Manual override (always allowed, even when frozen) ──
        if (manualNow) {
            // Snapshot ticks when manual STARTS (rising edge)
            if (!manualWasActive) {
                manualStartTicks = rawTicks;
            }

            double req = manualPower;
            if ((rawTicks >= TURRET_MAX_TICKS && req > 0) ||
                    (rawTicks <= TURRET_MIN_TICKS && req < 0)) {
                req = 0.0;
            }
            if (req > 1.0) req = 1.0;
            if (req < -1.0) req = -1.0;

            turretMotor.setPower(req);
            integral = 0.0;
            lastErrorDeg = 0.0;
            lastTimeMs = nowMs;
            lastAppliedPower = 0.0;
            manualWasActive = true;

            if (freezeMode) freezeTargetTicks = turretMotor.getCurrentPosition();
            publishTelemetry();
            return;
        }

        // ── When releasing manual: capture the angular offset ──
        if (manualWasActive) {
            int ticksMoved = turretMotor.getCurrentPosition() - manualStartTicks;
            manualOffsetRad += ticksToRadians(ticksMoved);

            integral = 0.0;
            lastErrorDeg = 0.0;
            lastTimeMs = nowMs;
            lastAppliedPower = 0.0;
        }
        manualWasActive = false;

        // ── Freeze hold (after homing or explicit freeze) ──
        if (freezeMode) {
            holdRawTicks(freezeTargetTicks);
            publishTelemetry();
            return;
        }

        // ══════════════════════════════════════
        //  AUTO AIMING — THE CORE LOGIC
        // ══════════════════════════════════════

        // 1. Get robot pose from Follower
        if (follower == null) {
            turretMotor.setPower(0.0);
            lastAppliedPower = 0.0;
            publishTelemetry();
            return;
        }

        Pose robotPose = follower.getPose();
        if (robotPose == null) {
            turretMotor.setPower(0.0);
            lastAppliedPower = 0.0;
            publishTelemetry();
            return;
        }

        // ── First-pose guard: skip the very first cycle to let the localizer settle.
        //    This prevents the turret from lurching to a stale/default heading on startup.
        if (!hasFirstPose) {
            hasFirstPose = true;
            lastTimeMs = nowMs;
            // Don't move the turret this cycle — just record that we have a pose now
            turretMotor.setPower(0.0);
            lastAppliedPower = 0.0;
            publishTelemetry();
            return;
        }

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeadingRad = robotPose.getHeading(); // Pedro: 0 = +X, CCW positive

        // 2. Compute field-frame bearing from robot → goal
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;
        double distToGoal = Math.sqrt(dx * dx + dy * dy);

        double fieldBearingRad;
        if (distToGoal >= MIN_GOAL_DIST) {
            fieldBearingRad = bearingRad(robotX, robotY, GOAL_X, GOAL_Y);
            lastValidDesiredRad = fieldBearingRad;
        } else {
            // Too close — hold last known aim direction to avoid wild spinning
            fieldBearingRad = lastValidDesiredRad;
        }

        // 3. Convert to turret-relative angle (+ manual offset from driver nudge)
        double turretDesiredRad = normalizeAngle(
                fieldBearingRad - robotHeadingRad + TURRET_FORWARD_OFFSET_RAD + manualOffsetRad
        );

        // ── 3.5 VELOCITY COMPENSATION ──
        // Compute bearing rate (how fast the angle to goal is changing)
        // and add a feedforward lead so the turret anticipates motion.
        double velCompDeg = 0.0;
        if (!Double.isNaN(lastBearingRad) && lastTimeMs > 0) {
            long dtMsVel = Math.max(1, nowMs - lastTimeMs);
            double dtVel = dtMsVel / 1000.0;

            // Rate of change of the field bearing (deg/sec)
            double bearingDelta = normalizeAngle(fieldBearingRad - lastBearingRad);
            double rawRateDegPerSec = Math.toDegrees(bearingDelta) / dtVel;

            // Low-pass filter to reject noise
            filteredBearingRateDegPerSec =
                    BEARING_RATE_FILTER * filteredBearingRateDegPerSec
                            + (1.0 - BEARING_RATE_FILTER) * rawRateDegPerSec;

            // Lead = gain × rate
            velCompDeg = VELOCITY_COMP_GAIN * filteredBearingRateDegPerSec;
        }
        lastBearingRad = fieldBearingRad;

        // Apply velocity lead to desired turret angle
        double velCompRad = Math.toRadians(velCompDeg);
        turretDesiredRad = normalizeAngle(turretDesiredRad + velCompRad);

        // 4. Convert to desired encoder ticks
        double desiredTicksDouble = radiansToTicks(turretDesiredRad);
        int desiredTicks = (int) Math.round(desiredTicksDouble);

        // Clamp to encoder limits
        if (desiredTicks > TURRET_MAX_TICKS) desiredTicks = TURRET_MAX_TICKS;
        if (desiredTicks < TURRET_MIN_TICKS) desiredTicks = TURRET_MIN_TICKS;

        // 5) Apply rightward damping by biasing desired target itself
        rawTicks = turretMotor.getCurrentPosition();

        int delta = desiredTicks - rawTicks;
        int desiredBiased = desiredTicks;

        // Match TurretController behavior:
        // rightward means positive delta in encoder tick space
        boolean rightDampActive = delta > 0 && Math.abs(delta) <= RIGHTWARD_DAMP_ERROR_WINDOW;
        if (rightDampActive) {
            double scale = Math.max(0.0, 1.0 - RIGHTWARD_ENCODER_DAMP);
            desiredBiased = rawTicks + (int) Math.round(delta * scale);
        }

        // Clamp biased desired to limits
        if (desiredBiased > TURRET_MAX_TICKS) desiredBiased = TURRET_MAX_TICKS;
        if (desiredBiased < TURRET_MIN_TICKS) desiredBiased = TURRET_MIN_TICKS;

        // Compute error in DEGREES (for PID)
        int errorTicks = desiredBiased - rawTicks;
        double ticksPerDeg = TICKS_PER_FULL_ROTATION / 360.0;
        double errorDeg = errorTicks / ticksPerDeg;

        // 6. PID
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        // Integral with deadband gating
        if (Math.abs(errorDeg) > DEADBAND_DEG) {
            integral += errorDeg * dt;
        } else {
            integral *= 0.9; // slowly decay when on-target
        }
        // Anti-windup clamp
        if (integral > INTEGRAL_CLAMP_DEG) integral = INTEGRAL_CLAMP_DEG;
        if (integral < -INTEGRAL_CLAMP_DEG) integral = -INTEGRAL_CLAMP_DEG;

        // Derivative
        double derivative = (errorDeg - lastErrorDeg) / Math.max(1e-4, dt);
        lastErrorDeg = errorDeg;

        // PID output
        double pidOut = KP * errorDeg + KI * integral + KD * derivative;

        // Zero output in deadband
        if (Math.abs(errorDeg) <= DEADBAND_DEG) pidOut = 0.0;

        // Clamp to max power
        if (pidOut > MAX_POWER) pidOut = MAX_POWER;
        if (pidOut < -MAX_POWER) pidOut = -MAX_POWER;

        // Power smoothing (EMA)
        double applied = POWER_SMOOTH * lastAppliedPower + (1.0 - POWER_SMOOTH) * pidOut;

        // 7. Enforce encoder hard limits
        if ((rawTicks >= TURRET_MAX_TICKS && applied > 0) ||
                (rawTicks <= TURRET_MIN_TICKS && applied < 0)) {
            applied = 0.0;
        }

        // 8. Apply!
        turretMotor.setPower(applied);
        lastAppliedPower = applied;

        // ── Store telemetry values ──
        tRobotX = robotX;
        tRobotY = robotY;
        tRobotHeadingDeg = Math.toDegrees(robotHeadingRad);
        tBearingDeg = Math.toDegrees(fieldBearingRad);
        tTurretDesiredDeg = Math.toDegrees(turretDesiredRad);
        tErrorDeg = errorDeg;
        tDistToGoal = distToGoal;
        tAppliedPower = applied;
        tEncoderTicks = rawTicks;
        tDesiredTicks = desiredBiased;
        tVelCompDeg = velCompDeg;

        publishTelemetry();
    }

    // ══════════════════════════════════════════════════
    //  TELEMETRY
    // ══════════════════════════════════════════════════

    public double getErrorDeg()          { return tErrorDeg; }
    public double getDistToGoal()        { return tDistToGoal; }
    public double getBearingDeg()        { return tBearingDeg; }
    public double getTurretDesiredDeg()  { return tTurretDesiredDeg; }
    public double getRobotHeadingDeg()   { return tRobotHeadingDeg; }
    public int    getEncoderTicks()      { return tEncoderTicks; }
    public int    getDesiredTicks()      { return tDesiredTicks; }
    public double getAppliedPower()      { return tAppliedPower; }
    public double getVelCompDeg()        { return tVelCompDeg; }


    private void publishTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("aim.robotPos", "(%.1f, %.1f)", tRobotX, tRobotY);
        telemetry.addData("aim.robotHead", "%.1f°", tRobotHeadingDeg);
        telemetry.addData("aim.bearing", "%.1f°", tBearingDeg);

        telemetry.addData("aim.dist", "%.1f in", tDistToGoal);
        telemetry.addData("aim.encoder", "%d → %d", tEncoderTicks, tDesiredTicks);
        telemetry.addData("aim.power", "%.3f", tAppliedPower);

        telemetry.addData("aim.mode", freezeMode ? "FREEZE" : (homingMode ? "HOMING" : "TRACKING"));
        telemetry.addData("aim.velComp", "%.1f °/s → %.1f° lead",
                filteredBearingRateDegPerSec,
                tVelCompDeg);
    }
}