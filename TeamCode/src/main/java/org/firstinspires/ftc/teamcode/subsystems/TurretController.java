package org.firstinspires.ftc.teamcode.subsystems;

/*
  TurretController.java
  ---------------------
  Encapsulates all turret rotation / tracking logic (PID + IMU feedforward + encoder mapping).
  Moved out of the OpMode to declutter secondexperimentalHORS.

  Summary of important behavior / tuning (matches the values you provided):
  - Encoder hard limits: TURRET_MIN_POS = -600, TURRET_MAX_POS = 600
  - TICKS_PER_RADIAN computed from encoder span:
      TICKS_PER_RADIAN = (TURRET_MAX_POS - TURRET_MIN_POS) / (2 * Math.PI)
    (maps full encoder span to 360°; if your turret travel is not full 360°, replace denominator
    with the real travel radians)
  - PID: KP = 0.15, KI = 0.0005, KD = 0.1
  - FF_GAIN = 0.025
  - POWER smoothing ALPHA = 0.97
  - DERIV_FILTER_ALPHA = 0.70
  - SMALL_DEADBAND_TICKS = 3
  - INTEGRAL_CLAMP = 200.0
  - Integral accumulation is gated by deadband, derivative is filtered, and applied power is smoothed.
  - Manual control is handled inside this class; the OpMode should call update(manualNow, manualPower).
  - The class exposes telemetry getters so the OpMode can display useful tuning data.
*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretController {

    // Hardware references
    private final DcMotor turretMotor;
    private final BNO055IMU imu;
    private final Telemetry telemetry; // optional, may be null

    // Turret encoder hard limits (as requested)
    public static final int TURRET_MIN_POS = -600;
    public static final int TURRET_MAX_POS = 600;

    // ticks-per-radian mapping
    private static final double BASE_TICKS_PER_RADIAN = TURRET_MAX_POS / Math.PI; // ~159.155
    private static final double TICKS_PER_RADIAN_SCALE = 2;
    private static final double TICKS_PER_RADIAN = BASE_TICKS_PER_RADIAN * TICKS_PER_RADIAN_SCALE;


    // PID & control gains (from your tuned values)
    private static final double TURRET_KP = 1.0;//0.20
    private static final double TURRET_KI = 0.09;
    private static final double TURRET_KD = 0.3;//0.16
    private static final double TURRET_MAX_POWER = 1.0;

    // Feedforward and smoothing/filtering
    private static final double FF_GAIN = 0.029;//0.28
    private static final double POWER_SMOOTH_ALPHA = 0.96;//0.96
    private static final double DERIV_FILTER_ALPHA = 0.40;//0.7

    // Deadband & anti-windup
    private static final int SMALL_DEADBAND_TICKS = 4;//3
    private static final double INTEGRAL_CLAMP = 200.0;

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

    // state for manual->auto transition detection
    private boolean manualActiveLast = false;

    // telemetry values (readable by the OpMode)
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;

    public TurretController(DcMotor turretMotor, BNO055IMU imu, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.telemetry = telemetry;

        // Make sure motor is configured appropriately outside this class (encoder reset etc.)
        captureReferences(); // initial capture
        resetPidState();
    }

    /**
     * Capture the current IMU heading and turret encoder as the reference points.
     * Call this when you want to (re)zero the turret's mapping to robot heading (e.g., after manual control).
     */
    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;
        turretEncoderReference = turretMotor.getCurrentPosition();
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
     * Main update method. Call from OpMode loop.
     * - If manualNow is true: the controller will apply manualPower (respecting hard limits), and PID state is reset.
     * - If manualNow is false: the controller will run automatic tracking using IMU heading + turret encoder mapping.
     *
     * @param manualNow whether operator control is active
     * @param manualPower requested manual power in [-1, 1] (only used if manualNow)
     */
    public void update(boolean manualNow, double manualPower) {
        long nowMs = System.currentTimeMillis();

        // detect transitions
        if (manualNow) {
            // Manual: apply manual power but enforce encoder hard limits
            applyManualPower(manualPower);
            // Reset PID state so no integral/derivative carryover when returning to auto
            turretIntegral = 0.0;
            lastErrorTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            manualActiveLast = true;
            // publish telemetry if present
            publishTelemetry();
            return;
        }

        // If we just transitioned from manual to auto, re-capture references
        if (manualActiveLast && !manualNow) {
            captureReferences();
            // reset timing so derivative doesn't blow up
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        // Auto control: compute desired ticks from heading delta
        double currentHeadingRad = getHeadingRadians();
        double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);

        // Compute angular velocity (for feedforward)
        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }

        // Save last heading for next iteration
        lastHeadingRad = currentHeadingRad;

        // Desired encoder ticks corresponding to heading change:
        double desiredTicksDouble = turretEncoderReference - headingDelta * TICKS_PER_RADIAN;
        int desiredTicks = (int)Math.round(desiredTicksDouble);

        // Clamp desired ticks to encoder physical limits
        if (desiredTicks > TURRET_MAX_POS) desiredTicks = TURRET_MAX_POS;
        if (desiredTicks < TURRET_MIN_POS) desiredTicks = TURRET_MIN_POS;

        int currentTicks = turretMotor.getCurrentPosition();
        int errorTicks = desiredTicks - currentTicks;

        // Timing for PID
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        // Integral gating: only integrate when error is outside small deadband
        if (Math.abs(errorTicks) > SMALL_DEADBAND_TICKS) {
            // If the sign changed, reduce integral to avoid "holding" overshoot
            if (lastErrorTicks != 0 && ((errorTicks > 0 && lastErrorTicks < 0) || (errorTicks < 0 && lastErrorTicks > 0))) {
                turretIntegral *= 0.5;
            }
            turretIntegral += errorTicks * dt;
        } else {
            // gentle decay while inside deadband
            turretIntegral *= 0.90;
        }

        // Clamp integral
        if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
        if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

        // Derivative (filtered)
        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = DERIV_FILTER_ALPHA * rawDerivative + (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

        // PID output (note Ki multiplies integral sum)
        double pidOut = TURRET_KP * errorTicks + TURRET_KI * turretIntegral + TURRET_KD * derivativeFiltered;

        // Feedforward: turret should oppose robot yaw rate (angularVel)
        double ff = -angularVel * FF_GAIN;

        double cmdPower = pidOut + ff;

        // Deadband to avoid tiny jittering moves
        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) {
            cmdPower = 0.0;
        }

        // Clamp command to motor power limits
        if (cmdPower > TURRET_MAX_POWER) cmdPower = TURRET_MAX_POWER;
        if (cmdPower < -TURRET_MAX_POWER) cmdPower = -TURRET_MAX_POWER;

        // Smooth applied power (exponential smoothing)
        double applied = POWER_SMOOTH_ALPHA * lastAppliedPower + (1.0 - POWER_SMOOTH_ALPHA) * cmdPower;

        // If at hard stop, block the command that would push further
        if ((currentTicks >= TURRET_MAX_POS && applied > 0.0) || (currentTicks <= TURRET_MIN_POS && applied < 0.0)) {
            applied = 0.0;
        }

        // Apply to motor
        turretMotor.setPower(applied);

        // Update states for next loop
        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;

        // Save telemetry values for external reading or logging
        lastDesiredTicks = desiredTicks;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;

        publishTelemetry();
    }

    private void applyManualPower(double manualPower) {
        int currentTicks = turretMotor.getCurrentPosition();

        // Enforce physical limits: if at limit, block further motion into the stop
        double requested = manualPower;
        if ((currentTicks >= TURRET_MAX_POS && requested > 0.0) || (currentTicks <= TURRET_MIN_POS && requested < 0.0)) {
            requested = 0.0;
        }

        // clamp to motor safe range
        if (requested > 1.0) requested = 1.0;
        if (requested < -1.0) requested = -1.0;

        turretMotor.setPower(requested);
        // reflect telemetry values
        lastDesiredTicks = turretEncoderReference;
        lastErrorReported = lastDesiredTicks - currentTicks;
        lastPidOut = 0.0;
        lastFf = 0.0;
    }

    /**
     * Return current heading reading (Z axis) in radians using the supplied IMU.
     * If imu is null, returns 0.0.
     */
    private double getHeadingRadians() {
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

    private void publishTelemetry() {
        if (telemetry == null) return;
//        telemetry.addData("turret.desired", lastDesiredTicks);
//        telemetry.addData("turret.error", lastErrorReported);
//        telemetry.addData("turret.pid", String.format("%.4f", lastPidOut));
//        telemetry.addData("turret.ff", String.format("%.4f", lastFf));
//        telemetry.addData("turret.applied", String.format("%.4f", lastAppliedPower));
    }
}
