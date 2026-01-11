package org.firstinspires.ftc.teamcode.tracking;

/*
  TurretController.java
  ---------------------
  Encapsulates all turret rotation / tracking logic (PID + IMU feedforward + encoder mapping).
  Moved out of the OpMode to declutter secondexperimentalHORS.

  Summary of important behavior / tuning (matches the values you provided):
  - Encoder hard limits: TURRET_MIN_POS = -900, TURRET_MAX_POS = 900
  - TICKS_PER_RADIAN computed from encoder span:
      TICKS_PER_RADIAN = (TURRET_MAX_POS - TURRET_MIN_POS) / (2 * Math.PI) * TICKS_PER_RADIAN_SCALE
    (maps full encoder span to 360°; if your turret travel is not full 360°, replace denominator
    with the real travel radians)
  - PID: KP = 1.15, KI = 0.0, KD = 0.22
  - FF_GAIN = 5.0
  - POWER smoothing ALPHA = 0.935
  - DERIV_FILTER_ALPHA = 1.25
  - SMALL_DEADBAND_TICKS = 14
  - INTEGRAL_CLAMP = 50.0
  - Integral accumulation is gated by deadband, derivative is filtered, and applied power is smoothed.
  - Manual control is handled inside this class; the OpMode should call update(manualNow, manualPower).
  - The class exposes telemetry getters so the OpMode can display useful tuning data.
*/

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

@Configurable
public class TurretController {

    // Hardware references
    private final DcMotor turretMotor;
    private final BNO055IMU imu;                    // Expansion Hub IMU fallback
    private final GoBildaPinpointDriver pinpoint;   // Preferred heading source
    private final Telemetry telemetry; // optional, may be null

    // Turret encoder hard limits (configurable)
    @Sorter(sort = 0)
    public static int TURRET_MIN_POS = -1000;

    @Sorter(sort = 1)
    public static int TURRET_MAX_POS = 1000;
    @Sorter(sort = 2)
    public static double TICKS_PER_RADIAN_SCALE = 0.9;

    // PID like config
    @Sorter(sort = 3)
    public static double TURRET_KP = 1.2;

    @Sorter(sort = 4)
    public static double TURRET_KI = 0.0;

    @Sorter(sort = 5)
    public static double TURRET_KD = 0.25;

    @Sorter(sort = 6)
    public static double TURRET_MAX_POWER = 1.0;

    // Feedforward and smoothing/filtering (configurable)
    @Sorter(sort = 7)
    public static double FF_GAIN = 5.0;

    @Sorter(sort = 8)
    public static double POWER_SMOOTH_ALPHA = 0.94;

    @Sorter(sort = 9)
    public static double DERIV_FILTER_ALPHA = 1.0;

    // Deadband & anti-windup (configurable)
    @Sorter(sort = 10)
    public static int SMALL_DEADBAND_TICKS = 4; // every 4 ticks change required

    @Sorter(sort = 11)
    public static double INTEGRAL_CLAMP = 50.0; // not used at all

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

    // telemetry values (readable by the OpMode)
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;

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
     * Cleanly disable turret tracking and stop the motor.
     */
    public void disable() {
        turretIntegral = 0.0;
        lastErrorTicks = 0;
        lastDerivative = 0.0;
        lastAppliedPower = 0.0;
        lastTimeMs = -1L;
        manualActiveLast = false;
        turretMotor.setPower(0.0);
    }

    /**
     * Main update method. Call from OpMode loop.
     * - If manualNow is true: the controller will apply manualPower (respecting hard limits), and PID state is reset.
     * - If manualNow is false: the controller will run automatic tracking using heading + turret encoder mapping.
     *
     * @param manualNow whether operator control is active
     * @param manualPower requested manual power in [-1, 1] (only used if manualNow)
     */
    public void update(boolean manualNow, double manualPower) {
        long nowMs = System.currentTimeMillis();

        // Pull latest config each loop so UI changes apply live
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

        // Derived mapping
        double ticksPerRad = ((maxPosCfg - minPosCfg) / (2.0 * Math.PI)) * ticksPerRadScaleCfg;

        // Current virtual encoder position
        int currentVirtualTicks = getVirtualEncoderPosition();

        // detect transitions
        if (manualNow) {
            // Manual: apply manual power but enforce virtual hard limits
            applyManualPower(manualPower, minPosCfg, maxPosCfg, currentVirtualTicks);
            // Reset PID state
            turretIntegral = 0.0;
            lastErrorTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            manualActiveLast = true;
            publishTelemetry();
            return;
        }

        // If we just transitioned from manual to auto, re-capture references
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

        int errorTicks = desiredVirtualTicks - currentVirtualTicks;

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
        lastDesiredTicks = desiredVirtualTicks;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;

        publishTelemetry();
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

    private void publishTelemetry() {
        if (telemetry == null) return;
        // Uncomment for debugging:
        // telemetry.addData("turret.desired", lastDesiredTicks);
         telemetry.addData("\n turret.error", lastErrorReported);
        // telemetry.addData("turret.pid", String.format("%.4f", lastPidOut));
        // telemetry.addData("turret.ff", String.format("%.4f", lastFf));
        // telemetry.addData("turret.applied", String.format("%.4f", lastAppliedPower));
         telemetry.addData("turret.virtualPos", getVirtualEncoderPosition());
         //telemetry.addData("\n turret.offset", encoderOffset);
    }
}