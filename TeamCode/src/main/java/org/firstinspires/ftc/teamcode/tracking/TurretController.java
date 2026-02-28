package org.firstinspires.ftc.teamcode.tracking;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

@Configurable
public class TurretController {

    // Hardware references
    private final DcMotor turretMotor;
    private final BNO055IMU imu;                    // Expansion Hub IMU fallback
    private final GoBildaPinpointDriver pinpoint;   // Preferred heading source
    private final Telemetry telemetry;              // optional, may be null

    // Optional reset trigger (e.g., magnetic limit switch). Used only when homing sweep is active.
    private BooleanSupplier encoderResetTrigger = null;

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

    // Rightward asymmetry control (biases target instead of slowing PID output)
    @Sorter(sort = 12)
    public static double RIGHTWARD_ENCODER_DAMP = 0.9;
    @Sorter(sort = 13)
    public static int RIGHTWARD_DAMP_ERROR_WINDOW = 50;

    // Homing sweep configuration (manual reset)
    @Sorter(sort = 14)
    public static int HOMING_AMPLITUDE_TICKS = 300;
    @Sorter(sort = 15)
    public static double HOMING_POWER = 0.5;
    @Sorter(sort = 16)
    public static int HOMING_TARGET_DEADBAND = 12;

    // Safety timeout (ms) to exit homing if switch never triggers
    @Sorter(sort = 17)
    public static long HOMING_TIMEOUT_MS = 3000;

    // Internal state
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Heading / encoder reference (used to compute desired ticks from IMU)
    private double headingReferenceRad = 0.0;
    private double lastHeadingRad = 0.0;
    private int turretEncoderReference = 0;

    // Encoder offset for "virtual" zero after captureReferences
    private int encoderOffset = 0;

    // state for manual->auto transition detection
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

    // telemetry values (readable by the OpMode)
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;
    private double lastHeadingDelta = 0.0;
    private double lastAngularVel = 0.0;
    private int lastDampedError = 0;

    /**
     * Preferred constructor: allows supplying a Pinpoint plus a fallback IMU.
     */
    public TurretController(DcMotor turretMotor, BNO055IMU imu, GoBildaPinpointDriver pinpoint, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;

        captureReferences(); // initial capture
        resetPidState();
    }

    /**
     * Backward-compatible constructor (no Pinpoint provided).
     */
    public TurretController(DcMotor turretMotor, BNO055IMU imu, Telemetry telemetry) {
        this(turretMotor, imu, null, telemetry);
    }

    /**
     * Allow the OpMode to provide a trigger that resets the turret encoder when true.
     * Example usage with a REV magnetic limit switch (active-low):
     *   controller.setEncoderResetTrigger(() -> !limitSwitch.getState());
     */
    public void setEncoderResetTrigger(BooleanSupplier trigger) {
        this.encoderResetTrigger = trigger;
    }

    /**
     * Command a manual homing sweep (oscillate between -amp and +amp) using a button.
     * Rising edge starts the sweep; if already frozen, rising edge unfreezes and resumes tracking.
     */
    public void commandHomingSweep(boolean homingButtonPressed) {
        if (homingButtonPressed && !homingCommandPrev) {
            if (freezeMode) {
                // Unfreeze: re-capture references and resume tracking
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
        freezeMode = false; // ensure we are not frozen while homing
        // clear PID-related history to avoid stale values
        turretIntegral = 0.0;
        lastDerivative = 0.0;
        lastAppliedPower = 0.0;
        lastTimeMs = System.currentTimeMillis();
    }

    /**
     * Get the current encoder position in "virtual" space (offset-adjusted).
     */
    private int getVirtualEncoderPosition() {
        return turretMotor.getCurrentPosition() - encoderOffset;
    }

    /**
     * Capture the current heading and turret encoder as the reference points.
     * This recenters the "virtual" encoder space so that the current position becomes 0.
     */
    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;

        // Offset so current raw position becomes virtual 0
        encoderOffset = turretMotor.getCurrentPosition();
        turretEncoderReference = 0; // virtual reference is zero
    }

    /**
     * Reset PID internal state (useful on mode switches).
     */
    public void resetPidState() {
        turretIntegral = 0.0;
        lastErrorTicks = 0;
        lastTimeMs = System.currentTimeMillis();
        lastAppliedPower = 0.0;
        lastDerivative = 0.0;
        manualActiveLast = false;
    }

    /**
     * Force-exit homing/freeze and recenter heading + encoder reference.
     * Useful for a “reset” button that should immediately resume tracking without homing.
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

        captureReferences();   // sets headingReferenceRad/lastHeadingRad and encoderOffset
        resetPidState();       // clears PID integrator/derivative history
        lastTimeMs = System.currentTimeMillis();
    }

    /**
     * Cleanly disable turret tracking and stop the motor.
     */
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

    /**
     * utility to drive toward raw encoder target.
     * Returns true if within deadband and motor is stopped.
     */
    public boolean driveToPosition(int targetTicks, int deadbandTicks, double powerMag) {
        int current = turretMotor.getCurrentPosition();
        double power;
        if (current < targetTicks - deadbandTicks) {
            power = Math.abs(powerMag);
        } else if (current > targetTicks + deadbandTicks) {
            power = -Math.abs(powerMag);
        } else {
            power = 0.0;
        }
        turretMotor.setPower(power);
        return power == 0.0;
    }

    /**
     * Hold a fixed encoder target (virtual ticks) using the same PID loop.
     */
    public void holdPositionTicks(int targetVirtualTicks) {
        long nowMs = System.currentTimeMillis();

        int minPosCfg = TURRET_MIN_POS;
        int maxPosCfg = TURRET_MAX_POS;
        double kpCfg = TURRET_KP;
        double kiCfg = TURRET_KI;
        double kdCfg = TURRET_KD;
        double maxPowerCfg = TURRET_MAX_POWER;
        double powerSmoothCfg = POWER_SMOOTH_ALPHA;
        double derivFilterCfg = DERIV_FILTER_ALPHA;
        int deadbandCfg = SMALL_DEADBAND_TICKS;
        double integralClampCfg = INTEGRAL_CLAMP;

        // Clamp target
        if (targetVirtualTicks > maxPosCfg) targetVirtualTicks = maxPosCfg;
        if (targetVirtualTicks < minPosCfg) targetVirtualTicks = minPosCfg;

        int currentVirtualTicks = getVirtualEncoderPosition();
        int errorTicks = targetVirtualTicks - currentVirtualTicks;

        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

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

        if (Math.abs(errorTicks) <= deadbandCfg) {
            pidOut = 0.0;
        }

        if (pidOut > maxPowerCfg) pidOut = maxPowerCfg;
        if (pidOut < -maxPowerCfg) pidOut = -maxPowerCfg;

        double applied = powerSmoothCfg * lastAppliedPower + (1.0 - powerSmoothCfg) * pidOut;

        if ((currentVirtualTicks >= maxPosCfg && applied > 0.0) || (currentVirtualTicks <= minPosCfg && applied < 0.0)) {
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
        lastDampedError = errorTicks;

        publishTelemetry();
    }

    /**
     * Main update method. Call from OpMode loop.
     * - Homing overrides everything until done.
     * - Freeze mode: holds position, no auto heading tracking, but manual still works.
     * - Manual always takes priority and can adjust even while frozen; the held target is updated to the manual position.
     */
    public void update(boolean manualNow, double manualPower) {
        long nowMs = System.currentTimeMillis();

        // --- Homing sweep overrides everything until done ---
        if (homingMode) {
            if (runHomingSweep()) {
                // homing finished this cycle; fall through so freeze/hold can engage
                nowMs = System.currentTimeMillis();
            } else {
                return; // still homing; skip normal control
            }
        }

        // Current virtual encoder position (used in multiple branches)
        int currentVirtualTicks = getVirtualEncoderPosition();

        // --- Manual branch (always allowed, even when frozen) ---
        if (manualNow) {
            applyManualPower(manualPower, TURRET_MIN_POS, TURRET_MAX_POS, currentVirtualTicks);
            // Reset PID state timing to avoid stale derivatives
            turretIntegral = 0.0;
            lastErrorTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            manualActiveLast = true;

            // If frozen, keep holding from the new manually-set position
            if (freezeMode) {
                freezeHoldTarget = getVirtualEncoderPosition();
            }
            publishTelemetry();
            return;
        }

        // --- Freeze/hold mode: bypass heading tracking, hold position ---
        if (freezeMode) {
            holdPositionTicks(freezeHoldTarget);
            return;
        }

        // --- Auto tracking branch (only when not frozen) ---
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

        // Derived mapping
        double ticksPerRad = ((maxPosCfg - minPosCfg) / (2.0 * Math.PI)) * ticksPerRadScaleCfg;

        // detect transitions from manual->auto
        if (manualActiveLast && !manualNow) {
            captureReferences();
            currentVirtualTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        // Auto control: compute desired ticks from heading delta
        double currentHeadingRad = getHeadingRadians();
        double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);

        // Angular velocity (for feedforward)
        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }

        // Save last heading
        lastHeadingRad = currentHeadingRad;

        // Desired ticks in virtual space
        double desiredTicksDouble = turretEncoderReference - headingDelta * ticksPerRad;
        int desiredVirtualTicks = (int) Math.round(desiredTicksDouble);

        // Clamp desired to limits
        if (desiredVirtualTicks > maxPosCfg) desiredVirtualTicks = maxPosCfg;
        if (desiredVirtualTicks < minPosCfg) desiredVirtualTicks = minPosCfg;

        // Rightward asymmetry by biasing the target itself
        int delta = desiredVirtualTicks - currentVirtualTicks;
        int desiredBiased = desiredVirtualTicks;
        boolean rightDampActive = delta > 0 && Math.abs(delta) <= rightDampWindowCfg;
        if (rightDampActive) {
            double scale = Math.max(0.0, 1.0 - rightDampCfg);
            desiredBiased = currentVirtualTicks + (int) Math.round(delta * scale);
        }

        // Clamp biased desired to limits
        if (desiredBiased > maxPosCfg) desiredBiased = maxPosCfg;
        if (desiredBiased < minPosCfg) desiredBiased = minPosCfg;

        int errorTicks = desiredBiased - currentVirtualTicks;

        // Timing
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

        // Clamp integral
        if (turretIntegral > integralClampCfg) turretIntegral = integralClampCfg;
        if (turretIntegral < -integralClampCfg) turretIntegral = -integralClampCfg;

        // Derivative (filtered)
        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = derivFilterCfg * rawDerivative + (1.0 - derivFilterCfg) * lastDerivative;

        // PID output
        double pidOut = kpCfg * errorTicks + kiCfg * turretIntegral + kdCfg * derivativeFiltered;

        // Feedforward: oppose robot yaw rate
        double ff = -angularVel * ffGainCfg;

        double cmdPower = pidOut + ff;

        // Deadband to avoid tiny jittering moves
        if (Math.abs(errorTicks) <= deadbandCfg) {
            cmdPower = 0.0;
        }

        // Clamp command to motor power limits
        if (cmdPower > maxPowerCfg) cmdPower = maxPowerCfg;
        if (cmdPower < -maxPowerCfg) cmdPower = -maxPowerCfg;

        // Smooth applied power
        double applied = powerSmoothCfg * lastAppliedPower + (1.0 - powerSmoothCfg) * cmdPower;

        // If at hard stop, block further
        if ((currentVirtualTicks >= maxPosCfg && applied > 0.0) || (currentVirtualTicks <= minPosCfg && applied < 0.0)) {
            applied = 0.0;
        }

        // Apply to motor
        turretMotor.setPower(applied);

        // Update states
        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;

        // Telemetry values
        lastDesiredTicks = desiredBiased;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;
        lastHeadingDelta = headingDelta;
        lastAngularVel = angularVel;
        lastDampedError = errorTicks;

        publishTelemetry();
    }

    /**
     * Homing sweep state machine. Returns true if homing finished (reset captured).
     * Stops immediately on limit switch and cuts power to avoid overshoot.
     */
    private boolean runHomingSweep() {
        // Immediate stop if limit switch trips
        boolean resetNow = encoderResetTrigger != null && encoderResetTrigger.getAsBoolean();
        if (resetNow) {
            turretMotor.setPower(0.0); // cut power first to prevent coasting past the switch
            try {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception ignored) {}
            captureReferences();   // realign virtual zero & heading reference
            resetPidState();       // clear PID state
            lastTimeMs = System.currentTimeMillis();
            lastAppliedPower = 0.0;
            lastDerivative = 0.0;

            // Enter freeze/hold after successful homing
            freezeMode = true;
            freezeHoldTarget = getVirtualEncoderPosition(); // typically 0 after captureReferences()
            homingMode = false;    // exit homing after successful reset
            return true;           // done homing
        }

        // Safety timeout: bail out if switch never triggers
        if (System.currentTimeMillis() - homingStartMs > HOMING_TIMEOUT_MS) {
            turretMotor.setPower(0.0);
            homingMode = false;
            // do not freeze; just exit homing
            return true;
        }

        int current = getVirtualEncoderPosition();

        // Simple sweep: flip direction when nearing amplitude
        if (homingDirectionPos && current >= homingTarget - HOMING_TARGET_DEADBAND) {
            homingDirectionPos = false;
            homingTarget = -HOMING_AMPLITUDE_TICKS;
        } else if (!homingDirectionPos && current <= homingTarget + HOMING_TARGET_DEADBAND) {
            homingDirectionPos = true;
            homingTarget = HOMING_AMPLITUDE_TICKS;
        }

        double power = homingDirectionPos ? HOMING_POWER : -HOMING_POWER;

        // Respect hard limits
        if ((current >= TURRET_MAX_POS && power > 0.0) || (current <= TURRET_MIN_POS && power < 0.0)) {
            power = 0.0;
        }

        turretMotor.setPower(power);

        // Minimal telemetry for homing
        lastDesiredTicks = homingTarget;
        lastErrorReported = homingTarget - current;
        lastPidOut = 0.0;
        lastFf = 0.0;
        lastHeadingDelta = 0.0;
        lastAngularVel = 0.0;
        lastDampedError = lastErrorReported;

        publishTelemetry();
        return false; // still homing
    }

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
        lastDampedError = lastErrorReported;
    }

    /**
     * Return current heading (Z) in radians. Prefer Pinpoint; fall back to BNO IMU; else 0.
     * NOTE: Pinpoint heading is negated to match the previous BNO055 sign convention.
     */
    private double getHeadingRadians() {
        if (pinpoint != null) {
            try {
                return -pinpoint.getHeading(AngleUnit.RADIANS); // invert to match prior BNO IMU convention
            } catch (Exception ignored) {
            }
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

    // --- Telemetry getters (OpMode can read these and put them on screen) ---
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
    public int getLastDampedError() { return lastDampedError; }

    private void publishTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("turret.desired", lastDesiredTicks);
        telemetry.addData("turret.virtualPos", getVirtualEncoderPosition());
        telemetry.addData("turret.rawPos", getRawPosition());
        telemetry.addData("turret.error", lastErrorReported);
        telemetry.addData("turret.dampedError", lastDampedError);
        telemetry.addData("turret.offset", encoderOffset);
        telemetry.addData("turret.freeze", freezeMode);
        telemetry.addData("turret.homing", homingMode);
    }
}