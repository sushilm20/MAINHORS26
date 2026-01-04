package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * PIDF flywheel controller (dual-motor capable) with backward-compatible shims for existing callers.
 * Supports configurable close/far RPM targets (2650/4500) that propagate across the codebase.
 */
@Configurable
public class FlywheelController {

    private final DcMotorEx shooter;
    private final DcMotor shooter2; // mirrors power (opposite direction)
    private final Telemetry telemetry; // nullable
    private final ElapsedTime timer = new ElapsedTime();

    // --- Panels-configurable constants (public static) ---
    @Sorter(sort = 0) public static double MAX_RPM = 6000.0;
    @Sorter(sort = 1) public static double TICKS_PER_REV = 28.0;

    @Sorter(sort = 2) public static double kP = 0.0025;
    @Sorter(sort = 3) public static double kI = 0.0;
    @Sorter(sort = 4) public static double kD = 0.0003;
    @Sorter(sort = 5) public static double kF = 1.0 / 6000.0; // feedforward: power per RPM

    @Sorter(sort = 6) public static double integralLimit = 150.0; // limit on integral sum (in RPM-error units)
    @Sorter(sort = 7) public static double derivativeAlpha = 0.85;  // low-pass (0..1), higher = smoother

    @Sorter(sort = 8) public static double closeRPM = 2650.0;
    @Sorter(sort = 9) public static double farRPM   = 4500.0;

    @Sorter(sort = 10) public static double rpmTolerance = 50.0; // “ready to shoot” window

    // Legacy aliases so existing code can keep compiling while we migrate callers
    @Sorter(sort = 11) public static double TARGET_RPM_CLOSE;
    @Sorter(sort = 12) public static double TARGET_RPM_FAR;
    @Sorter(sort = 13) public static double TARGET_TOLERANCE_RPM;

    // --- Internal state ---
    private double targetRpm = closeRPM;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastDerivativeEstimate = 0.0;
    private int lastPos = 0;
    private double currentRpm = 0.0;
    private double lastAppliedPower = 0.0;

    // legacy controls
    private boolean shooterOn = true;
    private boolean lastAtTarget = false;
    private boolean justReachedTargetFlag = false;
    private boolean leftTriggerLast = false;
    private double savedTargetBeforeTrigger = -1.0;
    private boolean savedShooterOnBeforeTrigger = false;

    public FlywheelController(DcMotor shooter, DcMotor shooter2, Telemetry telemetry) {
        if (!(shooter instanceof DcMotorEx)) {
            throw new IllegalArgumentException("Primary shooter must be a DcMotorEx");
        }
        this.shooter = (DcMotorEx) shooter;
        this.shooter2 = shooter2;
        this.telemetry = telemetry;

        try {
            this.shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("Flywheel.init", "primary cfg failed: " + e.getMessage());
        }

        if (this.shooter2 != null) {
            try {
                this.shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } catch (Exception e) {
                if (telemetry != null) telemetry.addData("Flywheel.init", "secondary cfg failed: " + e.getMessage());
            }
        }

        timer.reset();
        lastPos = this.shooter.getCurrentPosition();
    }

    // Backward-compatible ctor for existing single-motor call sites.
    public FlywheelController(DcMotor shooter, Telemetry telemetry) {
        this(shooter, null, telemetry);
    }

    public void setTargetRpm(double rpm) {
        if (rpm != targetRpm) {
            // reset integral on setpoint changes (anti-windup)
            integralSum = 0.0;
            lastError = 0.0;
            lastDerivativeEstimate = 0.0;
        }
        targetRpm = rpm;
    }

    public double getTargetRpm() { return targetRpm; }

    public boolean isAtSpeed() {
        double error = targetRpm - currentRpm;
        return Math.abs(error) <= rpmTolerance;
    }

    public void update() {
        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3; // avoid div/0

        double currentRpmNow = getCurrentRpm(dt);
        currentRpm = currentRpmNow;
        timer.reset();

        double error = targetRpm - currentRpmNow;

        // Integral with clamping
        integralSum += error * dt;
        double limit = integralLimit;
        if (integralSum > limit) integralSum = limit;
        if (integralSum < -limit) integralSum = -limit;

        // Derivative with low-pass on error change
        double rawDeriv = (error - lastError) / dt;
        double deriv = derivativeAlpha * lastDerivativeEstimate + (1.0 - derivativeAlpha) * rawDeriv;
        lastDerivativeEstimate = deriv;

        // Feedforward: scale target RPM to power
        double ff = kF * targetRpm; // expects kF ≈ 1 / maxRPM (tune)

        // PIDF output
        double out = ff + (kP * error) + (kI * integralSum) + (kD * deriv);

        // Clamp to [-1, 1]
        out = Math.max(-1.0, Math.min(1.0, out));

        if (!shooterOn) out = 0.0;

        // Apply to motors (shooter2 spins opposite)
        try {
            shooter.setPower(out);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("Flywheel.power", "primary setPower failed: " + e.getMessage());
        }

        if (shooter2 != null) {
            try {
                shooter2.setPower(-out);
            } catch (Exception e) {
                if (telemetry != null) telemetry.addData("Flywheel.power", "secondary setPower failed: " + e.getMessage());
            }
        }

        lastAppliedPower = out;
        lastError = error;

        // At-target detection (rising edge sets justReachedTargetFlag)
        boolean atTargetNow = Math.abs(error) <= rpmTolerance;
        if (atTargetNow && !lastAtTarget) {
            justReachedTargetFlag = true;
        }
        lastAtTarget = atTargetNow;

        // --- Single-line telemetry: target & current side by side, PIDF terms ---
        if (telemetry != null) {
            telemetry.addLine(String.format(
                    "Flywheel || tgt: %5.0f rpm || cur: %5.0f rpm || P: %+1.3f I: %+1.3f D: %+1.3f F: %+1.3f || out: %+1.3f || err: %+6.0f || dt: %.3f",
                    targetRpm,
                    currentRpmNow,
                    (kP * error),
                    (kI * integralSum),
                    (kD * deriv),
                    kF * targetRpm,
                    out,
                    error,
                    dt
            ));
        }
    }

    private double getCurrentRpm(double dtSeconds) {
        // Prefer built-in velocity if available; fallback to position diff
        double ticksPerSecond;
        try {
            ticksPerSecond = shooter.getVelocity(); // ticks/sec
            lastPos = shooter.getCurrentPosition();
        } catch (Exception e) {
            int pos = shooter.getCurrentPosition();
            int delta = pos - lastPos;
            lastPos = pos;
            double dt = (dtSeconds <= 0) ? 1e-3 : dtSeconds;
            ticksPerSecond = delta / dt;
        }
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }

    // Convenience: set close/far modes
    public void setCloseMode() { setTargetRpm(closeRPM); }
    public void setFarMode()   { setTargetRpm(farRPM); }

    // --- Legacy API shims (for existing callers) ---
    public void setTargetRPM(double rpm) { setTargetRpm(rpm); }
    public double getTargetRPM() { return getTargetRpm(); }
    public void adjustTargetRPM(double delta) { setTargetRpm(Math.max(0.0, targetRpm + delta)); }
    public void setModeFar(boolean far) { setTargetRpm(far ? farRPM : closeRPM); }
    public void toggleShooterOn() { shooterOn = !shooterOn; }
    public void setShooterOn(boolean on) { shooterOn = on; }
    public boolean isShooterOn() { return shooterOn; }
    public double getCurrentRPM() { return currentRpm; }
    public double getLastAppliedPower() { return lastAppliedPower; }
    public boolean isAtTarget() { return isAtSpeed(); }

    public boolean justReachedTarget() {
        if (justReachedTargetFlag) {
            justReachedTargetFlag = false;
            return true;
        }
        return false;
    }

    /**
     * Preserve the left-trigger behavior from the prior controller:
     * - on press: save previous target and set low intake target
     * - on release: restore previous target and shooter state
     */
    public void handleLeftTrigger(boolean leftTriggerNow) {
        if (leftTriggerNow && !leftTriggerLast) {
            savedTargetBeforeTrigger = targetRpm;
            savedShooterOnBeforeTrigger = shooterOn;
            setTargetRpm(40.0);
            shooterOn = true;
        } else if (!leftTriggerNow && leftTriggerLast) {
            if (savedTargetBeforeTrigger >= 0.0) {
                setTargetRpm(savedTargetBeforeTrigger);
                savedTargetBeforeTrigger = -1.0;
            }
            shooterOn = savedShooterOnBeforeTrigger;
            savedShooterOnBeforeTrigger = false;
        }
        leftTriggerLast = leftTriggerNow;
    }

    // Legacy overload to match old update signature
    public void update(long ignoredNowMs, boolean ignoredCalibPressed) {
        update();
    }

    // Runtime tuning of the tolerance for rumble/vibrate trigger
    public void setTargetToleranceRpm(double tolerance) {
        if (tolerance < 0) tolerance = 0;
        rpmTolerance = tolerance;
        TARGET_TOLERANCE_RPM = tolerance; // keep dashboard in sync if changed programmatically
    }

    public double getTargetToleranceRpm() {
        return rpmTolerance;
    }
}
    static {
        TARGET_RPM_CLOSE = closeRPM;
        TARGET_RPM_FAR = farRPM;
        TARGET_TOLERANCE_RPM = rpmTolerance;
    }
