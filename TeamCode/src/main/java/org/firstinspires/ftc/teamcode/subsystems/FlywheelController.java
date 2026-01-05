package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * PIDF flywheel controller (dual-motor capable) with backward-compatible shims for existing callers.
 * Supports configurable close/far RPM targets (2650/4500) that propagate across the codebase.
 */
@Configurable
public class FlywheelController {

    private final DcMotorEx shooter;
    private final DcMotor shooter2; // mirrors power (opposite direction via motor direction)
    private final Telemetry telemetry; // nullable
    private final VoltageSensor voltageSensor; // nullable
    private final ElapsedTime timer = new ElapsedTime();

    // --- Panels-configurable constants (public static) ---
    @Sorter(sort = 0) public static double MAX_RPM = 6000.0;
    @Sorter(sort = 1) public static double TICKS_PER_REV = 28.0;

    @Sorter(sort = 2) public static double kP = 0.00145;
    @Sorter(sort = 3) public static double kI = 0.0027;//maybe i do
    @Sorter(sort = 4) public static double kD = 0.00002;
    @Sorter(sort = 5) public static double kF = 1.72; //gang im so goated

    @Sorter(sort = 6) public static double integralLimit = 50; // limit on integral sum (in RPM-error units)
    @Sorter(sort = 7) public static double derivativeAlpha = 0.9;  // low-pass (0..1), higher = smoother

    @Sorter(sort = 8) public static double closeRPM = 2630;
    @Sorter(sort = 9) public static double farRPM   = 3500.0;

    @Sorter(sort = 10) public static double rpmTolerance = 50.0; // “ready to shoot” window

    // Feedforward reference points (ticks/sec at reference voltage)
    @Sorter(sort = 11) public static double ffReferenceVoltage = 13.0;   //so at 13 volts we run 4930 ticks per second
    @Sorter(sort = 12) public static double ffReferenceMaxTicksPerSec = 4930; // ticks/sec @ reference

    // Optional smoothing (1.0 = no change; lower = more smoothing)
    @Sorter(sort = 13) public static double rpmFilterAlpha = 0.72;       // low-pass on measured RPM
    @Sorter(sort = 14) public static double powerSmoothingAlpha = 0.5;  // low-pass on applied power

    // Legacy variable mapped to new
    public static double TARGET_RPM_CLOSE;
    public static double TARGET_RPM_FAR;
    public static double TARGET_TOLERANCE_RPM;

    // static initializer must be inside the class
    static {
        TARGET_RPM_CLOSE = closeRPM;
        TARGET_RPM_FAR = farRPM;
        TARGET_TOLERANCE_RPM = rpmTolerance;
    }

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

    public FlywheelController(DcMotor shooter, DcMotor shooter2, Telemetry telemetry, VoltageSensor voltageSensor) {
        if (!(shooter instanceof DcMotorEx)) {
            throw new IllegalArgumentException("Primary shooter must be a DcMotorEx");
        }
        this.shooter = (DcMotorEx) shooter;
        this.shooter2 = shooter2;
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;

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
                this.shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // mirror mode (no encoder)
            } catch (Exception e) {
                if (telemetry != null) telemetry.addData("Flywheel.init", "secondary cfg failed: " + e.getMessage());
            }
        }

        timer.reset();
        lastPos = this.shooter.getCurrentPosition();
    }

    // Backward-compatible ctor for existing single-motor call sites.
    public FlywheelController(DcMotor shooter, Telemetry telemetry, VoltageSensor voltageSensor) {
        this(shooter, null, telemetry, voltageSensor);
    }

    // Backward-compatible ctor without voltage sensor (feedforward will use a fallback voltage)
    public FlywheelController(DcMotor shooter, DcMotor shooter2, Telemetry telemetry) {
        this(shooter, shooter2, telemetry, null);
    }

    public FlywheelController(DcMotor shooter, Telemetry telemetry) {
        this(shooter, null, telemetry, null);
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
        // Low-pass filter on measured RPM (alpha=1 => no filtering)
        currentRpm = rpmFilterAlpha * currentRpmNow + (1.0 - rpmFilterAlpha) * currentRpm;

        timer.reset();

        double error = targetRpm - currentRpm;

        // Integral with clamping
        integralSum += error * dt;
        double limit = integralLimit;
        if (integralSum > limit) integralSum = limit;
        if (integralSum < -limit) integralSum = -limit;

        // Derivative with low-pass on error change
        double rawDeriv = (error - lastError) / dt;
        double deriv = derivativeAlpha * lastDerivativeEstimate + (1.0 - derivativeAlpha) * rawDeriv;
        lastDerivativeEstimate = deriv;

        // Dynamic feedforward based on battery voltage and measured max ticks/sec
        double voltage = getBatteryVoltage();
        double maxTicksPerSec = (voltage / ffReferenceVoltage) * ffReferenceMaxTicksPerSec;
        if (maxTicksPerSec < 1e-3) maxTicksPerSec = 1e-3;
        double targetTicksPerSec = (targetRpm * TICKS_PER_REV) / 60.0;
        double ff = (targetTicksPerSec / maxTicksPerSec) * kF; // kF is a gain on the fraction

        // PIDF output
        double out = ff + (kP * error) + (kI * integralSum) + (kD * deriv);

        // Clamp to [-1, 1]
        out = Math.max(-1.0, Math.min(1.0, out));

        if (!shooterOn) out = 0.0;

        // Optional power smoothing (alpha=1 => no smoothing)
        double smoothedOut = powerSmoothingAlpha * out + (1.0 - powerSmoothingAlpha) * lastAppliedPower;

        // Apply to motors (shooter2 uses motor direction for opposite spin)
        try {
            shooter.setPower(smoothedOut);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData("Flywheel.power", "primary setPower failed: " + e.getMessage());
        }

        if (shooter2 != null) {
            try {
                shooter2.setPower(smoothedOut); // same magnitude; opposite comes from hardware direction
            } catch (Exception e) {
                if (telemetry != null) telemetry.addData("Flywheel.power", "secondary setPower failed: " + e.getMessage());
            }
        }

        lastAppliedPower = smoothedOut;
        lastError = error;

        // At-target detection (rising edge sets justReachedTargetFlag)
        boolean atTargetNow = Math.abs(error) <= rpmTolerance;
        if (atTargetNow && !lastAtTarget) {
            justReachedTargetFlag = true;
        }
        lastAtTarget = atTargetNow;

        // --- Single-line telemetry: target & current side by side, PIDF terms ---

    }

    private double getBatteryVoltage() {
        try {
            if (voltageSensor != null) {
                double v = voltageSensor.getVoltage();
                if (v > 1e-3) return v;
            }
        } catch (Exception ignored) {}
        return 12.0; // fallback
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