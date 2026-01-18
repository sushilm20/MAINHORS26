package org.firstinspires.ftc.teamcode.tracking;

/*
  TurretController.java
  ---------------------
  Encapsulates all turret rotation / tracking logic (PID + IMU feedforward + encoder mapping).
  Now: reference drift eliminated—"home" heading always lines up with "home" encoder reliably!
  If you return to the reference heading, the turret will always try to use the reference encoder.

  Main logic:
  - At reference moment, we store both heading and encoder.
  - The turret always aims for:
      TARGET_ENCODER = referenceEncoder - (currentHeading - referenceHeading) * ticksPerRadian
  - This means that returning to referenceHeading always means returning to referenceEncoder.
  - Eliminates drift, perfectly matches human intuition!
  - NEW: Ignores heading changes within ±2° (deadzone).
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

    // Hardware refs
    private final DcMotor turretMotor;
    private final BNO055IMU imu;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;

    // Configurable limits/tuning
    @Sorter(sort = 0)
    public static int TURRET_MIN_POS = -1000;
    @Sorter(sort = 1)
    public static int TURRET_MAX_POS = 1000;
    @Sorter(sort = 2)
    public static double TICKS_PER_RADIAN_SCALE = 0.9;

    @Sorter(sort = 3)
    public static double TURRET_KP = 1.1;
    @Sorter(sort = 4)
    public static double TURRET_KI = 3;
    @Sorter(sort = 5)
    public static double TURRET_KD = 0.22;

    @Sorter(sort = 6)
    public static double TURRET_MAX_POWER = 1.0;

    // Feedforward, smoothing, filtering
    @Sorter(sort = 7)
    public static double FF_GAIN = 7.0;

    @Sorter(sort = 8)
    public static double POWER_SMOOTH_ALPHA = 0.93;

    @Sorter(sort = 9)
    public static double DERIV_FILTER_ALPHA = 1.0;

    // Deadband/anti-windup
    @Sorter(sort = 10)
    public static int SMALL_DEADBAND_TICKS = 4;
    @Sorter(sort = 11)
    public static double INTEGRAL_CLAMP = 5.0;

    /** NEW: Ignore heading changes less than this threshold (in radians). Default = ~2deg = 0.0349 rad **/
    @Sorter (sort = 12)
    public static double MIN_HEADING_CHANGE_RAD = Math.toRadians(2.0);

    // PID/internal state
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Driftless reference snapshot
    private double referenceHeadingRad = 0.0;     // heading (rads) at reference
    private int referenceEncoderPos = 0;          // encoder ticks at reference

    // Used for angular velocity/FF
    private double lastHeadingRad = 0.0;
    private boolean manualActiveLast = false;

    // Telemetry variables
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;

    // To preserve legacy vars for external callers (not strictly necessary now)
    private int encoderOffset = 0;
    private double headingReferenceRad = 0.0;
    private int turretEncoderReference = 0;

    public TurretController(DcMotor turretMotor, BNO055IMU imu, GoBildaPinpointDriver pinpoint, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;

        captureReferences(); // initial capture
        resetPidState();
    }

    public TurretController(DcMotor turretMotor, BNO055IMU imu, Telemetry telemetry) {
        this(turretMotor, imu, null, telemetry);
    }

    /**
     * Capture the current heading and turret encoder as the reference points.
     * This also updates legacy fields for API/telemetry compat.
     */
    public void captureReferences() {
        referenceHeadingRad = getHeadingRadians();
        referenceEncoderPos = turretMotor.getCurrentPosition();
        // for compatible usage with old fields:
        headingReferenceRad = referenceHeadingRad;
        lastHeadingRad = referenceHeadingRad;
        encoderOffset = referenceEncoderPos;
        turretEncoderReference = 0; // always virtualized to 0
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
     * Simple utility to drive toward a raw encoder target.
     * Returns true if within deadband and motor is stopped.
     */
    public boolean driveToPosition(int targetTicks, int deadbandTicks, double powerMag) {
        int current = turretMotor.getCurrentPosition();
        double power = 0.0;
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
     * Main update method. Call from OpMode loop.
     * - If manualNow is true: the controller will apply manualPower (respecting hard limits), and PID state is reset.
     * - If manualNow is false: the controller will run automatic tracking using reference heading/encoder.
     *
     * @param manualNow whether operator control is active
     * @param manualPower requested manual power in [-1, 1] (only used if manualNow)
     */
    public void update(boolean manualNow, double manualPower) {
        long nowMs = System.currentTimeMillis();

        // Derived mapping
        double ticksPerRad = ((TURRET_MAX_POS - TURRET_MIN_POS) / (2.0 * Math.PI)) * TICKS_PER_RADIAN_SCALE;

        int currentEncoderPos = turretMotor.getCurrentPosition();

        // Manual mode
        if (manualNow) {
            applyManualPower(manualPower, TURRET_MIN_POS, TURRET_MAX_POS, currentEncoderPos - referenceEncoderPos);
            turretIntegral = 0.0;
            lastErrorTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            manualActiveLast = true;
            publishTelemetry();
            return;
        }

        // Transition from manual to auto
        if (manualActiveLast && !manualNow) {
            captureReferences();
            currentEncoderPos = turretMotor.getCurrentPosition();
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        // Main logic: driftless + heading deadzone
        double currentHeadingRad = getHeadingRadians();
        double headingDelta = normalizeAngle(currentHeadingRad - referenceHeadingRad);

        int desiredEncoderPos;
        if (Math.abs(headingDelta) < MIN_HEADING_CHANGE_RAD) {
            // Ignore small heading changes!
            desiredEncoderPos = currentEncoderPos;
        } else {
            desiredEncoderPos = (int) Math.round(referenceEncoderPos - headingDelta * ticksPerRad);
        }

        // Clamp to hard limits
        if (desiredEncoderPos > TURRET_MAX_POS) desiredEncoderPos = TURRET_MAX_POS;
        if (desiredEncoderPos < TURRET_MIN_POS) desiredEncoderPos = TURRET_MIN_POS;

        int errorTicks = desiredEncoderPos - currentEncoderPos;

        // Timing
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        // Angular velocity (for FF)
        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, dt);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }
        lastHeadingRad = currentHeadingRad;

        // PID state logic
        if (Math.abs(errorTicks) > SMALL_DEADBAND_TICKS) {
            if (lastErrorTicks != 0 && ((errorTicks > 0 && lastErrorTicks < 0) || (errorTicks < 0 && lastErrorTicks > 0))) {
                turretIntegral *= 0.5;
            }
            turretIntegral += errorTicks * dt;
        } else {
            turretIntegral *= 0.90;
        }

        // Clamp integral
        if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
        if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

        // Derivative (filtered)
        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = DERIV_FILTER_ALPHA * rawDerivative + (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

        double pidOut = TURRET_KP * errorTicks + TURRET_KI * turretIntegral + TURRET_KD * derivativeFiltered;
        double ff = -angularVel * FF_GAIN;

        double cmdPower = pidOut + ff;

        // Disable tiny moves for deadzone or within encoder deadband
        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS || Math.abs(headingDelta) < MIN_HEADING_CHANGE_RAD) {
            cmdPower = 0.0;
        }

        // Clamp to max power
        if (cmdPower > TURRET_MAX_POWER) cmdPower = TURRET_MAX_POWER;
        if (cmdPower < -TURRET_MAX_POWER) cmdPower = -TURRET_MAX_POWER;

        // Smoothing
        double applied = POWER_SMOOTH_ALPHA * lastAppliedPower + (1.0 - POWER_SMOOTH_ALPHA) * cmdPower;

        // Clamp at hard stops
        if ((currentEncoderPos >= TURRET_MAX_POS && applied > 0.0) || (currentEncoderPos <= TURRET_MIN_POS && applied < 0.0)) {
            applied = 0.0;
        }

        turretMotor.setPower(applied);

        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;

        lastDesiredTicks = desiredEncoderPos - referenceEncoderPos;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;

        publishTelemetry();
    }

    private void applyManualPower(double manualPower, int minPosCfg, int maxPosCfg, int tickDeltaFromRef) {
        double requested = manualPower;
        int currentVirtualTicks = tickDeltaFromRef;
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
            } catch (Exception ignored) {}
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
    public int getVirtualPosition() { return turretMotor.getCurrentPosition() - referenceEncoderPos; }
    public int getRawPosition() { return turretMotor.getCurrentPosition(); }

    private void publishTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("\n turret.error", lastErrorReported);
        telemetry.addData("\n Turret Encoder: ", getRawPosition());
    }
}