package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
  Flywheel.java
  --------------
  Changes in this version:
  - Exposes isAtTarget() so the OpMode can rumble continuously while the flywheel
    is within the tolerance window.
  - Keeps the justReachedTarget() one-shot API (unchanged) in case other code uses it.
  - Keeps runtime getter/setter for the tolerance (setTargetToleranceRpm/getTargetToleranceRpm).

  Usage:
  - Call update(nowMs, calibPressed) every loop.
  - Call handleLeftTrigger(...) to preserve the intake temporary-target behavior.
  - Call isAtTarget() from the OpMode and trigger rumble repeatedly while true to get continuous vibration.
*/

public class Flywheel {

    private final DcMotor shooter;
    private final Telemetry telemetry; // nullable

    // --- Tunable constants ---
    private static final double MAX_RPM = 200.0;
    private static final double TICKS_PER_REV = 537.6;

    private static final double K_P = 0.0003;
    private static final double EMA_ALPHA = 0.35; // smoothing for measured RPM
    private static final double DEFAULT_RPM_SCALE = 0.78;

    private static final double TARGET_RPM_CLOSE = 90.0;
    private static final double TARGET_RPM_FAR   = 140.0;

    // runtime tolerance used for "at target" detection (vibration). Default increased per request.
    private double targetToleranceRpm = 10.0; // you can change via setTargetToleranceRpm(...)

    // --- Internal state ---
    private double currentRPM = 0.0;
    private int lastShooterPosition = 0;
    private long lastShooterTime = -1L;

    private double targetRPM;
    private boolean shooterOn = true;

    private double rpmScale = DEFAULT_RPM_SCALE;

    // Left-trigger / intake temporary-target support
    private boolean leftTriggerLast = false;
    private double savedTargetRPMBeforeLeftTrigger = -1.0;
    private boolean savedShooterOnBeforeLeftTrigger = false;

    // calibration button edge detect
    private boolean lastCalibPressed = false;

    // "just reached target" one-shot
    private boolean lastAtTarget = false;
    private boolean justReachedTargetFlag = false;

    // last applied motor power (for telemetry)
    private double lastAppliedPower = 0.0;

    public Flywheel(DcMotor shooterMotor, Telemetry telemetry) {
        this.shooter = shooterMotor;
        this.telemetry = telemetry;
        this.targetRPM = TARGET_RPM_CLOSE;
        this.shooterOn = true;

        // ensure shooter motor mode is expected (OpMode generally sets this)
        this.lastShooterPosition = shooter.getCurrentPosition();
        this.lastShooterTime = System.currentTimeMillis();
    }

    /**
     * Primary periodic update. Call from OpMode loop with current time and calibration button state.
     *
     * @param nowMs       current timestamp in ms
     * @param calibPressed whether the calibration button (your 'y') is currently pressed
     */
    public void update(long nowMs, boolean calibPressed) {
        // RPM measurement
        int shooterCurrentPosition = shooter.getCurrentPosition();
        long now = nowMs;
        long deltaTimeMs = (lastShooterTime < 0) ? 1 : Math.max(1, now - lastShooterTime);
        int deltaTicks = shooterCurrentPosition - lastShooterPosition;
        double ticksPerSec = (deltaTicks * 1000.0) / deltaTimeMs;
        double measuredRPMRaw = (ticksPerSec / TICKS_PER_REV) * 60.0;
        double measuredRPMScaled = measuredRPMRaw * rpmScale;

        // exponential smoothing for current RPM
        currentRPM = (1.0 - EMA_ALPHA) * currentRPM + EMA_ALPHA * measuredRPMScaled;
        if (currentRPM < 0.0) currentRPM = 0.0;

        // Save for next iteration
        lastShooterPosition = shooterCurrentPosition;
        lastShooterTime = now;

        // Calibration: on rising edge of calibration button, compute and apply rpmScale
        if (calibPressed && !lastCalibPressed) {
            if (Math.abs(measuredRPMRaw) >= 1.0) {
                double candidateScale = targetRPM / measuredRPMRaw;
                candidateScale = Math.max(0.2, Math.min(3.0, candidateScale));
                rpmScale = candidateScale;
                if (telemetry != null) telemetry.addData("Flywheel.Calib", String.format("scale=%.3f", rpmScale));
            } else {
                if (telemetry != null) telemetry.addData("Flywheel.Calib", "raw RPM too small");
            }
        }
        lastCalibPressed = calibPressed;

        // PID-like control (simple P + feedforward)
        double ff = targetRPM / Math.max(1.0, MAX_RPM);
        double error = targetRPM - currentRPM;
        double pTerm = K_P * error;
        double shooterPower = ff + pTerm;

        // clamp commanded power to [0,1] and only enable when shooterOn
        if (shooterPower > 1.0) shooterPower = 1.0;
        if (shooterPower < 0.0) shooterPower = 0.0;

        double applied = shooterOn ? shooterPower : 0.0;
        shooter.setPower(applied);
        lastAppliedPower = applied;

        // At-target detection (rising edge sets justReachedTargetFlag) using runtime tolerance
        boolean atTargetNow = Math.abs(targetRPM - currentRPM) <= targetToleranceRpm;
        if (atTargetNow && !lastAtTarget) {
            justReachedTargetFlag = true;
        }
        lastAtTarget = atTargetNow;

        // telemetry
        if (telemetry != null) {
            telemetry.addData("fly.Current RPM/n", String.format("%.1f", currentRPM));
            telemetry.addData("fly.targetRPM/n", String.format("%.1f", targetRPM));
            telemetry.addData("fly.power/n", String.format("%.3f", lastAppliedPower));
//            telemetry.addData("fly.scale", String.format("%.3f", rpmScale));
//            telemetry.addData("fly.tolerance", String.format("%.2f", targetToleranceRpm));
//            telemetry.addData("fly.atTarget", atTargetNow);
        }
    }

    /**
     * Preserve the left-trigger behavior:
     * - on press: save previous target and set low intake target
     * - on release: restore previous target and shooter state
     */
    public void handleLeftTrigger(boolean leftTriggerNow) {
        if (leftTriggerNow && !leftTriggerLast) {
            // pressed: save and set low target
            savedTargetRPMBeforeLeftTrigger = targetRPM;
            savedShooterOnBeforeLeftTrigger = shooterOn;
            targetRPM = 40.0;
            shooterOn = true;
        } else if (!leftTriggerNow && leftTriggerLast) {
            // released: restore
            if (savedTargetRPMBeforeLeftTrigger >= 0.0) {
                targetRPM = savedTargetRPMBeforeLeftTrigger;
                savedTargetRPMBeforeLeftTrigger = -1.0;
            }
            shooterOn = savedShooterOnBeforeLeftTrigger;
            savedShooterOnBeforeLeftTrigger = false;
        }
        leftTriggerLast = leftTriggerNow;
    }

    // external controls / helpers

    public void setTargetRPM(double rpm) { targetRPM = rpm; }
    public double getTargetRPM() { return targetRPM; }

    public void adjustTargetRPM(double delta) { targetRPM = Math.max(0.0, targetRPM + delta); }

    public void setModeFar(boolean far) { targetRPM = far ? TARGET_RPM_FAR : TARGET_RPM_CLOSE; }

    public void toggleShooterOn() { shooterOn = !shooterOn; }
    public void setShooterOn(boolean on) { shooterOn = on; }
    public boolean isShooterOn() { return shooterOn; }

    public double getCurrentRPM() { return currentRPM; }
    public double getLastAppliedPower() { return lastAppliedPower; }

    /**
     * Returns true once when the flywheel just reached the target (rising edge). The flag is cleared on read.
     */
    public boolean justReachedTarget() {
        if (justReachedTargetFlag) {
            justReachedTargetFlag = false;
            return true;
        }
        return false;
    }

    /**
     * Returns true whenever the current RPM is within the configured tolerance of the target.
     * Use this from the OpMode to trigger continuous controller vibration while true.
     */
    public boolean isAtTarget() {
        return Math.abs(targetRPM - currentRPM) <= targetToleranceRpm;
    }

    // Optional: getters for telemetry-friendly values
    public double getRpmScale() { return rpmScale; }
    public double getFpPercent() { return (MAX_RPM > 0) ? (targetRPM / MAX_RPM) : 0.0; }

    // Runtime tuning of the tolerance for rumble/vibrate trigger
    public void setTargetToleranceRpm(double tolerance) {
        if (tolerance < 0) tolerance = 0;
        this.targetToleranceRpm = tolerance;
    }

    public double getTargetToleranceRpm() {
        return this.targetToleranceRpm;
    }
}