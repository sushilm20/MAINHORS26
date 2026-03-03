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
 * PinpointTurretAimer — Pure Pinpoint-Based Field Aiming
 *
 *  SETUP:
 *   1. Before init, physically point turret straight forward.
 *   2. Zero the encoder → 0 ticks = turret aligned with chassis.
 *
 *  EVERY LOOP:
 *   1. turretFieldAngle = robotHeading + (encoderTicks * ENCODER_SIGN) / ticksPerDeg
 *   2. desiredFieldAngle = bearing(robotPos → goalPos)
 *   3. error = desired - current → PID → motor power
 *
 *  BEARING (StackExchange):
 *   θ_hat = atan2(goalX - robotX, goalY - robotY)
 *   θ = θ_hat < 0 ? θ_hat + 360 : θ_hat
 */
@Configurable
public class PinpointTurretAimer {

    private final DcMotor turretMotor;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;
    private BooleanSupplier encoderResetTrigger = null;

    // ── Goal (field inches) ──
    @Sorter(sort = 0) public static double GOAL_X = 14.0;
    @Sorter(sort = 1) public static double GOAL_Y = 134.0;

    // ── Encoder geometry ──
    // -1000 to +1000 = 2000 ticks for a full 360°
    @Sorter(sort = 2) public static double TICKS_PER_FULL_ROTATION = 2000.0;
    // Positive ticks = turret goes RIGHT, but we want positive = LEFT (CCW),
    // so ENCODER_SIGN = -1
    @Sorter(sort = 3) public static double ENCODER_SIGN = -1.0;

    // ── Encoder hard limits ──
    @Sorter(sort = 4) public static int TURRET_MIN_TICKS = -1000;
    @Sorter(sort = 5) public static int TURRET_MAX_TICKS = 1000;

    // ── PID (in degrees) ──
    @Sorter(sort = 6)  public static double KP = 0.015;
    @Sorter(sort = 7)  public static double KI = 0.0;
    @Sorter(sort = 8)  public static double KD = 0.003;
    @Sorter(sort = 9)  public static double MAX_POWER = 1.0;
    @Sorter(sort = 10) public static double DEADBAND_DEG = 1.5;
    @Sorter(sort = 11) public static double INTEGRAL_CLAMP_DEG = 100.0;
    @Sorter(sort = 12) public static double POWER_SMOOTH = 0.8;

    // ── Heading offset (if pinpoint 0° ≠ field 0°) ──
    @Sorter(sort = 13) public static double HEADING_OFFSET_DEG = 0.0;

    // ── Min distance to goal ──
    @Sorter(sort = 14) public static double MIN_GOAL_DIST = 3.0;

    // ── Homing ──
    @Sorter(sort = 15) public static int HOMING_AMP_TICKS = 300;
    @Sorter(sort = 16) public static double HOMING_POWER = 0.5;
    @Sorter(sort = 17) public static int HOMING_DEADBAND = 12;
    @Sorter(sort = 18) public static long HOMING_TIMEOUT_MS = 3000;

    // ── PID state ──
    private double integral = 0.0;
    private double lastErrorDeg = 0.0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private boolean manualWasActive = false;

    // ── Homing state ──
    private boolean homingMode = false;
    private boolean homingBtnPrev = false;
    private boolean homingDirPos = true;
    private int homingTarget = HOMING_AMP_TICKS;
    private long homingStartMs = 0L;

    // ── Freeze state ──
    private boolean freezeMode = false;
    private int freezeTargetTicks = 0;

    // ── Last valid desired (close to goal fallback) ──
    private double lastValidDesiredDeg = 0.0;

    // ── Telemetry readouts ──
    private double tRobotHeadDeg, tRobotX, tRobotY;
    private double tTurretFieldDeg, tDesiredFieldDeg, tErrorDeg;
    private double tDistToGoal, tAppliedPower;
    private int tEncoderTicks;

    // ══════════════════════════════════════════════════
    //  Constructor
    // ══════════════════════════════════════════════════

    public PinpointTurretAimer(DcMotor turretMotor,
                               GoBildaPinpointDriver pinpoint,
                               Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
    }

    public void setEncoderResetTrigger(BooleanSupplier trigger) {
        this.encoderResetTrigger = trigger;
    }

    public void setGoal(double x, double y) {
        GOAL_X = x;
        GOAL_Y = y;
    }

    // ══════════════════════════════════════════════════
    //  Core math
    // ══════════════════════════════════════════════════

    /**
     * Bearing from A to B in degrees [0, 360).
     * 0° = +Y ("north"), clockwise positive.
     *
     * θ_hat = atan2(bx - ax, by - ay)
     * θ = θ_hat < 0 ? θ_hat + 360 : θ_hat
     */
    private static double bearingDeg(double ax, double ay, double bx, double by) {
        double thetaRad = Math.atan2(bx - ax, by - ay);
        double thetaDeg = Math.toDegrees(thetaRad);
        if (thetaDeg < 0.0) thetaDeg += 360.0;
        return thetaDeg;
    }

    /**
     * Where is the turret pointing on the field right now?
     *
     * turretFieldAngle = robotHeading + encoderDegrees
     * encoderDegrees = (rawTicks * ENCODER_SIGN) / ticksPerDeg
     */
    private double getTurretFieldAngleDeg(double robotHeadingDeg) {
        int rawTicks = turretMotor.getCurrentPosition();
        double ticksPerDeg = TICKS_PER_FULL_ROTATION / 360.0;
        double encoderDeg = (rawTicks * ENCODER_SIGN) / ticksPerDeg;
        return robotHeadingDeg + encoderDeg;
    }

    private static double normalizeTo360(double deg) {
        deg = deg % 360.0;
        if (deg < 0.0) deg += 360.0;
        return deg;
    }

    /**
     * Shortest path from current to target, in [-180, +180].
     * Positive = target is clockwise from current.
     */
    private static double shortestAngleDiff(double targetDeg, double currentDeg) {
        double diff = targetDeg - currentDeg;
        while (diff > 180.0)   diff -= 360.0;
        while (diff <= -180.0) diff += 360.0;
        return diff;
    }

    // ══════════════════════════════════════════════════
    //  Reset / Disable
    // ══════════════════════════════════════════════════

    public void zeroEncoder() {
        try {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception ignored) {}
        clearPid();
    }

    public void fullReset() {
        homingMode = false;
        freezeMode = false;
        zeroEncoder();
    }

    private void clearPid() {
        integral = 0.0;
        lastErrorDeg = 0.0;
        lastTimeMs = System.currentTimeMillis();
        lastAppliedPower = 0.0;
        manualWasActive = false;
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

    // ══════════════════════════════════════════════════
    //  Homing sweep
    // ══════════════════════════════════════════════════

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

    private boolean runHomingSweep() {
        if (encoderResetTrigger != null && encoderResetTrigger.getAsBoolean()) {
            turretMotor.setPower(0.0);
            zeroEncoder();
            freezeMode = true;
            freezeTargetTicks = 0;
            homingMode = false;
            return true;
        }

        if (System.currentTimeMillis() - homingStartMs > HOMING_TIMEOUT_MS) {
            turretMotor.setPower(0.0);
            homingMode = false;
            return true;
        }

        int current = turretMotor.getCurrentPosition();

        if (homingDirPos && current >= homingTarget - HOMING_DEADBAND) {
            homingDirPos = false;
            homingTarget = -HOMING_AMP_TICKS;
        } else if (!homingDirPos && current <= homingTarget + HOMING_DEADBAND) {
            homingDirPos = true;
            homingTarget = HOMING_AMP_TICKS;
        }

        double power = homingDirPos ? HOMING_POWER : -HOMING_POWER;
        if ((current >= TURRET_MAX_TICKS && power > 0) ||
                (current <= TURRET_MIN_TICKS && power < 0)) {
            power = 0.0;
        }

        turretMotor.setPower(power);
        return false;
    }

    // ══════════════════════════════════════════════════
    //  Freeze hold (simple P on raw ticks)
    // ══════════════════════════════════════════════════

    private void holdRawTicks(int targetTicks) {
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

    // ════════════════════���═════════════════════════════
    //  Main update
    // ══════════════════════════════════════════════════

    public void update(boolean manualNow, double manualPower) {
        long nowMs = System.currentTimeMillis();

        // Homing overrides everything
        if (homingMode) {
            if (runHomingSweep()) {
                nowMs = System.currentTimeMillis();
            } else {
                publishTelemetry();
                return;
            }
        }

        int rawTicks = turretMotor.getCurrentPosition();

        // Manual override
        if (manualNow) {
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

        // Coming back from manual — just clear PID
        if (manualWasActive) {
            integral = 0.0;
            lastErrorDeg = 0.0;
            lastTimeMs = nowMs;
            lastAppliedPower = 0.0;
        }
        manualWasActive = false;

        // Freeze hold
        if (freezeMode) {
            holdRawTicks(freezeTargetTicks);
            publishTelemetry();
            return;
        }

        // ══════════════════════════════════════
        //  AUTO AIMING
        // ══════════════════════════════════════

        // 1. Read robot state from pinpoint
        double robotHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES) + HEADING_OFFSET_DEG;
        Pose2D pos = pinpoint.getPosition();
        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);

        // 2. Where is the turret pointing? (field degrees)
        double turretFieldDeg = getTurretFieldAngleDeg(robotHeadingDeg);

        // 3. Where SHOULD it point? (bearing to goal)
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;
        double distToGoal = Math.sqrt(dx * dx + dy * dy);

        double desiredFieldDeg;
        if (distToGoal >= MIN_GOAL_DIST) {
            desiredFieldDeg = bearingDeg(robotX, robotY, GOAL_X, GOAL_Y);
            lastValidDesiredDeg = desiredFieldDeg;
        } else {
            desiredFieldDeg = lastValidDesiredDeg;
        }

        // 4. Error (shortest path)
        double errorDeg = shortestAngleDiff(desiredFieldDeg, normalizeTo360(turretFieldDeg));

        // 5. PID
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        if (Math.abs(errorDeg) > DEADBAND_DEG) {
            integral += errorDeg * dt;
        } else {
            integral *= 0.9;
        }
        if (integral > INTEGRAL_CLAMP_DEG) integral = INTEGRAL_CLAMP_DEG;
        if (integral < -INTEGRAL_CLAMP_DEG) integral = -INTEGRAL_CLAMP_DEG;

        double derivative = (errorDeg - lastErrorDeg) / Math.max(1e-4, dt);
        lastErrorDeg = errorDeg;

        double pidOut = KP * errorDeg + KI * integral + KD * derivative;

        if (Math.abs(errorDeg) <= DEADBAND_DEG) pidOut = 0.0;
        if (pidOut > MAX_POWER) pidOut = MAX_POWER;
        if (pidOut < -MAX_POWER) pidOut = -MAX_POWER;

        double applied = POWER_SMOOTH * lastAppliedPower + (1.0 - POWER_SMOOTH) * pidOut;

        // 6. Enforce encoder limits
        rawTicks = turretMotor.getCurrentPosition();
        if ((rawTicks >= TURRET_MAX_TICKS && applied > 0) ||
                (rawTicks <= TURRET_MIN_TICKS && applied < 0)) {
            applied = 0.0;
        }

        // 7. Apply
        turretMotor.setPower(applied);
        lastAppliedPower = applied;

        // Telemetry
        tRobotHeadDeg = robotHeadingDeg;
        tRobotX = robotX;
        tRobotY = robotY;
        tTurretFieldDeg = normalizeTo360(turretFieldDeg);
        tDesiredFieldDeg = desiredFieldDeg;
        tErrorDeg = errorDeg;
        tDistToGoal = distToGoal;
        tAppliedPower = applied;
        tEncoderTicks = rawTicks;

        publishTelemetry();
    }

    // ══════════════════════════════════════════════════
    //  Telemetry
    // ══════════════════════════════════════════════════

    public double getErrorDeg()        { return tErrorDeg;}
    public double getDistToGoal()      { return tDistToGoal;}
    public double getTurretFieldDeg()  { return tTurretFieldDeg;}
    public double getDesiredFieldDeg() { return tDesiredFieldDeg;}
    public double getRobotHeading()    { return tRobotHeadDeg;}
    public int    getEncoderTicks()    { return tEncoderTicks;}
    public double getAppliedPower()    { return tAppliedPower;}
    public boolean isHomingMode()      { return homingMode;}
    public boolean isFreezeMode()      { return freezeMode;}

    private void publishTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("aim.robotHead", "%.1f°", tRobotHeadDeg);
        telemetry.addData("aim.robotPos", "(%.1f, %.1f)", tRobotX, tRobotY);
        telemetry.addData("aim.turretField", "%.1f°", tTurretFieldDeg);
        telemetry.addData("aim.desired", "%.1f°", tDesiredFieldDeg);
        telemetry.addData("aim.error", "%.1f°", tErrorDeg);
        telemetry.addData("aim.dist", "%.1f in", tDistToGoal);
        telemetry.addData("aim.encoder", tEncoderTicks);
        telemetry.addData("aim.power", "%.3f", tAppliedPower);
        telemetry.addData("aim.mode",
                freezeMode ? "FREEZE" : (homingMode ? "HOMING" : "AIM"));
    }
}