package org.firstinspires.ftc.teamcode.tracking;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.BooleanSupplier;

/**
 * PinpointTurretAimer — Pinpoint-Only Field Aiming
 *
 * Uses ONLY the GoBilda Pinpoint for both position and heading.
 * No Pedro Pathing pose, no coordinate frame mixing.
 *
 * ═══════════════════════════════════════════════════════
 *  HOW IT WORKS (from the resource):
 * ═══════════════════════════════════════════════════════
 *
 *  1. Before initializing, you zero the turret straight forward.
 *
 *  2. You know ticks per full rotation → ticksPerDegree = totalTicks / 360.
 *
 *  3. The turret's current FIELD-FACING angle is:
 *       turretFieldAngle = robotHeading + (encoderTicks / ticksPerDegree)
 *     (robotHeading from pinpoint, encoderTicks from the turret motor)
 *
 *  4. The DESIRED field angle is the bearing from the robot to the goal:
 *       desiredAngle = atan2(goalX - robotX, goalY - robotY)
 *     (atan2 with X in numerator, Y in denominator = bearing from north,
 *      matching FTC standard where 0° faces the red-goal wall)
 *
 *  5. Error = desiredAngle - turretFieldAngle → PID drives this to zero.
 *
 *  That's it. No "capture references". No virtual encoder offsets.
 *  The pinpoint is the single source of truth for position + heading.
 *  The raw encoder is the single source of truth for turret-to-chassis angle.
 *
 * ═══════════════════════════════════════════════════════
 *  FTC COORDINATE CONVENTION:
 * ═══════════════════════════════════════════════════════
 *  - Pinpoint gives heading where 0 = facing a specific wall
 *  - We use getHeading(DEGREES) directly — no negation, no conversion
 *  - Bearing: atan2(dx, dy) in degrees (StackExchange formula)
 *    This gives 0° = north (+Y direction), CW positive
 *  - If your field has 0° = +X, adjust HEADING_OFFSET_DEG
 *
 * ═══════════════════════════════════════════════════════
 *  ENCODER SIGN CONVENTION:
 * ═══════════════════════════════════════════════════════
 *  - Positive encoder ticks = turret rotates LEFT (CCW when viewed from above)
 *    → turret field angle INCREASES
 *  - If your turret is wired the other way, flip ENCODER_SIGN to -1
 */
@Configurable
public class PinpointTurretAimer {

    // ==================== Hardware ====================
    private final DcMotor turretMotor;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;

    private BooleanSupplier encoderResetTrigger = null;

    // ==================== Goal (field coordinates in inches) ====================
    @Sorter(sort = 0) public static double GOAL_X = 14.0;
    @Sorter(sort = 1) public static double GOAL_Y = 134.0;

    // ==================== Encoder geometry ====================
    // Total encoder ticks for a full 360° rotation of the turret
    @Sorter(sort = 2) public static double TICKS_PER_FULL_ROTATION = 2000.0 / 0.87;
    // +1 if positive ticks = CCW (left), -1 if positive ticks = CW (right)
    @Sorter(sort = 3) public static double ENCODER_SIGN = 1.0;

    // ==================== Encoder limits (raw ticks from zero) ====================
    @Sorter(sort = 4) public static int TURRET_MIN_TICKS = -1000;
    @Sorter(sort = 5) public static int TURRET_MAX_TICKS = 1000;

    // ==================== PID ====================
    @Sorter(sort = 6)  public static double KP = 0.02;   // power per degree of error
    @Sorter(sort = 7)  public static double KI = 0.0;
    @Sorter(sort = 8)  public static double KD = 0.004;
    @Sorter(sort = 9)  public static double MAX_POWER = 1.0;
    @Sorter(sort = 10) public static double DEADBAND_DEG = 1.5;  // degrees
    @Sorter(sort = 11) public static double INTEGRAL_CLAMP = 100.0;  // degrees*seconds
    @Sorter(sort = 12) public static double POWER_SMOOTH_ALPHA = 0.85;

    // ==================== Heading offset ====================
    // If the pinpoint's 0° doesn't match the FTC standard (0° = +X),
    // set this so that pinpointHeading + HEADING_OFFSET = FTC standard heading
    @Sorter(sort = 13) public static double HEADING_OFFSET_DEG = 0.0;

    // ==================== Min goal distance ====================
    @Sorter(sort = 14) public static double MIN_GOAL_DIST_INCHES = 3.0;

    // ==================== Homing ====================
    @Sorter(sort = 15) public static int HOMING_AMPLITUDE_TICKS = 300;
    @Sorter(sort = 16) public static double HOMING_POWER = 0.5;
    @Sorter(sort = 17) public static int HOMING_DEADBAND = 12;
    @Sorter(sort = 18) public static long HOMING_TIMEOUT_MS = 3000;

    // ==================== Derived (computed once) ====================
    private double ticksPerDegree;

    // ==================== PID state ====================
    private double integral = 0.0;
    private double lastErrorDeg = 0.0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;

    // ==================== Manual state ====================
    private boolean manualActiveLast = false;

    // ==================== Homing state ====================
    private boolean homingMode = false;
    private boolean homingCommandPrev = false;
    private boolean homingDirectionPos = true;
    private int homingTarget = HOMING_AMPLITUDE_TICKS;
    private long homingStartMs = 0L;

    // ==================== Freeze state ====================
    private boolean freezeMode = false;
    private int freezeHoldTicks = 0;

    // ==================== Last valid bearing (for close-to-goal) ====================
    private double lastValidDesiredDeg = 0.0;

    // ==================== Telemetry readouts ====================
    private double tRobotHeadingDeg = 0.0;
    private double tRobotX = 0.0;
    private double tRobotY = 0.0;
    private double tTurretFieldDeg = 0.0;
    private double tDesiredFieldDeg = 0.0;
    private double tErrorDeg = 0.0;
    private double tDistToGoal = 0.0;
    private double tAppliedPower = 0.0;
    private int tEncoderTicks = 0;

    // ==================== Constructor ====================

    public PinpointTurretAimer(DcMotor turretMotor, GoBildaPinpointDriver pinpoint, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
        recomputeDerived();
    }

    private void recomputeDerived() {
        ticksPerDegree = TICKS_PER_FULL_ROTATION / 360.0;
    }

    public void setEncoderResetTrigger(BooleanSupplier trigger) {
        this.encoderResetTrigger = trigger;
    }

    public void setGoal(double x, double y) {
        GOAL_X = x;
        GOAL_Y = y;
    }

    // ==================== Core: get turret's field-facing angle ====================

    /**
     * turretFieldAngle = robotHeading + encoderAngle
     * where encoderAngle = (rawTicks * ENCODER_SIGN) / ticksPerDegree
     */
    private double getTurretFieldAngleDeg(double robotHeadingDeg) {
        int rawTicks = turretMotor.getCurrentPosition();
        double encoderAngleDeg = (rawTicks * ENCODER_SIGN) / ticksPerDegree;
        return robotHeadingDeg + encoderAngleDeg;
    }

    // ==================== Core: get desired field angle (bearing to goal) ====================

    /**
     * Bearing from robot to goal using atan2(dx, dy).
     * Returns degrees, 0° = +Y direction (FTC "north"), CW positive.
     * Adjust by HEADING_OFFSET if needed.
     *
     * Reference: https://math.stackexchange.com/questions/1596513
     */
    private double getBearingToGoalDeg(double robotX, double robotY) {
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;
        // atan2(dx, dy) gives bearing from +Y axis, CW positive
        double bearingRad = Math.atan2(dx, dy);
        return Math.toDegrees(bearingRad);
    }

    // ==================== Read pinpoint ====================

    private double getRobotHeadingDeg() {
        return pinpoint.getHeading(AngleUnit.DEGREES) + HEADING_OFFSET_DEG;
    }

    private double getRobotX() {
        Pose2D pos = pinpoint.getPosition();
        return pos.getX(DistanceUnit.INCH);
    }

    private double getRobotY() {
        Pose2D pos = pinpoint.getPosition();
        return pos.getY(DistanceUnit.INCH);
    }

    // ==================== Normalize to [-180, 180] ====================

    private static double normalizeDeg(double deg) {
        while (deg <= -180.0) deg += 360.0;
        while (deg > 180.0) deg -= 360.0;
        return deg;
    }

    // ==================== Reset ====================

    /**
     * Zero the turret encoder. Call this when the turret is physically straight forward.
     * After this, encoder = 0 means turret aligned with chassis.
     */
    public void zeroTurretEncoder() {
        try {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception ignored) {}
        resetPidState();
    }

    public void resetPidState() {
        integral = 0.0;
        lastErrorDeg = 0.0;
        lastTimeMs = System.currentTimeMillis();
        lastAppliedPower = 0.0;
        manualActiveLast = false;
    }

    /**
     * Full reset: zero encoder, exit homing/freeze, clear PID.
     */
    public void fullReset() {
        homingMode = false;
        freezeMode = false;
        zeroTurretEncoder();
    }

    public void disable() {
        homingMode = false;
        freezeMode = false;
        integral = 0.0;
        lastErrorDeg = 0.0;
        lastAppliedPower = 0.0;
        lastTimeMs = -1L;
        turretMotor.setPower(0.0);
    }

    // ==================== Homing ====================

    public void commandHomingSweep(boolean buttonPressed) {
        if (buttonPressed && !homingCommandPrev) {
            if (freezeMode) {
                freezeMode = false;
                resetPidState();
            } else {
                homingMode = true;
                homingDirectionPos = true;
                homingTarget = HOMING_AMPLITUDE_TICKS;
                homingStartMs = System.currentTimeMillis();
                freezeMode = false;
                lastAppliedPower = 0.0;
                lastTimeMs = System.currentTimeMillis();
            }
        }
        homingCommandPrev = buttonPressed;
    }

    private boolean runHomingSweep() {
        // Check limit switch
        boolean resetNow = encoderResetTrigger != null && encoderResetTrigger.getAsBoolean();
        if (resetNow) {
            turretMotor.setPower(0.0);
            zeroTurretEncoder();
            freezeMode = true;
            freezeHoldTicks = 0;
            homingMode = false;
            return true;
        }

        // Timeout
        if (System.currentTimeMillis() - homingStartMs > HOMING_TIMEOUT_MS) {
            turretMotor.setPower(0.0);
            homingMode = false;
            return true;
        }

        int current = turretMotor.getCurrentPosition();

        if (homingDirectionPos && current >= homingTarget - HOMING_DEADBAND) {
            homingDirectionPos = false;
            homingTarget = -HOMING_AMPLITUDE_TICKS;
        } else if (!homingDirectionPos && current <= homingTarget + HOMING_DEADBAND) {
            homingDirectionPos = true;
            homingTarget = HOMING_AMPLITUDE_TICKS;
        }

        double power = homingDirectionPos ? HOMING_POWER : -HOMING_POWER;

        if ((current >= TURRET_MAX_TICKS && power > 0) || (current <= TURRET_MIN_TICKS && power < 0)) {
            power = 0.0;
        }

        turretMotor.setPower(power);
        return false;
    }

    // ==================== Hold position (freeze PID on encoder ticks) ====================

    private void holdPositionTicks(int targetTicks) {
        int current = turretMotor.getCurrentPosition();
        int errorTicks = targetTicks - current;

        double power = 0.0;
        if (Math.abs(errorTicks) > 3) {
            power = TurretController.TURRET_KP * errorTicks;
            if (power > MAX_POWER) power = MAX_POWER;
            if (power < -MAX_POWER) power = -MAX_POWER;
        }

        if ((current >= TURRET_MAX_TICKS && power > 0) || (current <= TURRET_MIN_TICKS && power < 0)) {
            power = 0.0;
        }

        turretMotor.setPower(power);
    }

    // ==================== Main update ====================

    /**
     * Call every loop iteration.
     *
     * @param manualNow   true if driver is manually overriding
     * @param manualPower power to apply during manual override [-1, 1]
     */
    public void update(boolean manualNow, double manualPower) {
        recomputeDerived(); // in case dashboard changed TICKS_PER_FULL_ROTATION
        long nowMs = System.currentTimeMillis();

        // ── Homing overrides everything ──
        if (homingMode) {
            if (runHomingSweep()) {
                nowMs = System.currentTimeMillis();
            } else {
                publishTelemetry();
                return;
            }
        }

        int rawTicks = turretMotor.getCurrentPosition();

        // ── Manual override ──
        if (manualNow) {
            double requested = manualPower;
            if ((rawTicks >= TURRET_MAX_TICKS && requested > 0) || (rawTicks <= TURRET_MIN_TICKS && requested < 0)) {
                requested = 0.0;
            }
            if (requested > 1.0) requested = 1.0;
            if (requested < -1.0) requested = -1.0;

            turretMotor.setPower(requested);

            // Reset PID so auto doesn't jerk on transition
            integral = 0.0;
            lastErrorDeg = 0.0;
            lastTimeMs = nowMs;
            lastAppliedPower = 0.0;
            manualActiveLast = true;

            if (freezeMode) {
                freezeHoldTicks = turretMotor.getCurrentPosition();
            }

            publishTelemetry();
            return;
        }

        // ── If coming back from manual, just reset PID (no "capture" needed!) ──
        if (manualActiveLast) {
            integral = 0.0;
            lastErrorDeg = 0.0;
            lastTimeMs = nowMs;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        // ── Freeze/hold ──
        if (freezeMode) {
            holdPositionTicks(freezeHoldTicks);
            publishTelemetry();
            return;
        }

        // ══════════════════════════════════════════════════
        // AUTO AIMING — the entire logic in ~15 lines
        // ══════════════════════════════════════════════════

        // Step 1: Read robot state from pinpoint
        double robotHeadingDeg = getRobotHeadingDeg();
        double robotX = getRobotX();
        double robotY = getRobotY();

        // Step 2: Where is the turret currently pointing? (field angle)
        double turretFieldDeg = getTurretFieldAngleDeg(robotHeadingDeg);

        // Step 3: Where SHOULD it point? (bearing from robot to goal)
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;
        double distToGoal = Math.sqrt(dx * dx + dy * dy);

        double desiredFieldDeg;
        if (distToGoal >= MIN_GOAL_DIST_INCHES) {
            desiredFieldDeg = getBearingToGoalDeg(robotX, robotY);
            lastValidDesiredDeg = desiredFieldDeg;
        } else {
            desiredFieldDeg = lastValidDesiredDeg;
        }

        // Step 4: Error = desired - actual (normalize to [-180, 180])
        double errorDeg = normalizeDeg(desiredFieldDeg - turretFieldDeg);

        // Step 5: PID on the error
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        // Integral
        if (Math.abs(errorDeg) > DEADBAND_DEG) {
            integral += errorDeg * dt;
        } else {
            integral *= 0.9;
        }
        if (integral > INTEGRAL_CLAMP) integral = INTEGRAL_CLAMP;
        if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP;

        // Derivative
        double derivative = (errorDeg - lastErrorDeg) / Math.max(1e-4, dt);
        lastErrorDeg = errorDeg;

        // PID output
        double pidOut = KP * errorDeg + KI * integral + KD * derivative;

        // Deadband
        if (Math.abs(errorDeg) <= DEADBAND_DEG) {
            pidOut = 0.0;
        }

        // Clamp
        if (pidOut > MAX_POWER) pidOut = MAX_POWER;
        if (pidOut < -MAX_POWER) pidOut = -MAX_POWER;

        // Smooth
        double applied = POWER_SMOOTH_ALPHA * lastAppliedPower + (1.0 - POWER_SMOOTH_ALPHA) * pidOut;

        // Step 6: Enforce encoder hard limits
        if ((rawTicks >= TURRET_MAX_TICKS && applied > 0) || (rawTicks <= TURRET_MIN_TICKS && applied < 0)) {
            applied = 0.0;
        }

        // Step 7: Apply!
        turretMotor.setPower(applied);
        lastAppliedPower = applied;

        // ── Store telemetry ──
        tRobotHeadingDeg = robotHeadingDeg;
        tRobotX = robotX;
        tRobotY = robotY;
        tTurretFieldDeg = turretFieldDeg;
        tDesiredFieldDeg = desiredFieldDeg;
        tErrorDeg = errorDeg;
        tDistToGoal = distToGoal;
        tAppliedPower = applied;
        tEncoderTicks = rawTicks;

        publishTelemetry();
    }

    // ==================== Telemetry ====================

    public double getErrorDeg()        { return tErrorDeg; }
    public double getDistToGoal()      { return tDistToGoal; }
    public double getTurretFieldDeg()  { return tTurretFieldDeg; }
    public double getDesiredFieldDeg() { return tDesiredFieldDeg; }
    public double getRobotHeading()    { return tRobotHeadingDeg; }
    public int    getEncoderTicks()    { return tEncoderTicks; }
    public double getAppliedPower()    { return tAppliedPower; }
    public boolean isHomingMode()      { return homingMode; }
    public boolean isFreezeMode()      { return freezeMode; }

    private void publishTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("aim.robotHead", "%.1f°", tRobotHeadingDeg);
        telemetry.addData("aim.robotPos", "(%.1f, %.1f)", tRobotX, tRobotY);
        telemetry.addData("aim.turretField", "%.1f°", tTurretFieldDeg);
        telemetry.addData("aim.desired", "%.1f°", tDesiredFieldDeg);
        telemetry.addData("aim.error", "%.1f°", tErrorDeg);
        telemetry.addData("aim.dist", "%.1f in", tDistToGoal);
        telemetry.addData("aim.encoder", tEncoderTicks);
        telemetry.addData("aim.power", "%.3f", tAppliedPower);
        telemetry.addData("aim.mode", freezeMode ? "FREEZE" : (homingMode ? "HOMING" : "AIM"));
    }
}