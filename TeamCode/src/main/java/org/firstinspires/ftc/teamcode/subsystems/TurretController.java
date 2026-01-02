package org.firstinspires.ftc.teamcode.subsystems;

/*
  TurretController.java
  ---------------------
  Encapsulates all turret rotation / tracking logic (PID + IMU feedforward + encoder mapping).
  Moved out of the OpMode to declutter secondexperimentalHORS.
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
    private final BNO055IMU imu;
    private final GoBildaPinpointDriver pinpoint; // optional, preferred if present
    private final Telemetry telemetry; // optional, may be null

    // Turret encoder hard limits (configurable)
    @Sorter(sort = 0)
    public static int TURRET_MIN_POS = -900;
    @Sorter(sort = 1)
    public static int TURRET_MAX_POS = 900;

    // ticks-per-radian mapping (configurable scale; base is derived each loop)
    @Sorter(sort = 2)
    public static double TICKS_PER_RADIAN_SCALE = 1.0;

    // PID & control gains (configurable)
    @Sorter(sort = 3)
    public static double TURRET_KP = 1.15;
    @Sorter(sort = 4)
    public static double TURRET_KI = 0.0;
    @Sorter(sort = 5)
    public static double TURRET_KD = 0.22;
    @Sorter(sort = 6)
    public static double TURRET_MAX_POWER = 1.0;

    // Feedforward and smoothing/filtering (configurable)
    @Sorter(sort = 7)
    public static double FF_GAIN = 5.0;
    @Sorter(sort = 8)
    public static double POWER_SMOOTH_ALPHA = 0.935;
    @Sorter(sort = 9)
    public static double DERIV_FILTER_ALPHA = 1.25;

    // Deadband & anti-windup (configurable)
    @Sorter(sort = 10)
    public static int SMALL_DEADBAND_TICKS = 14;
    @Sorter(sort = 11)
    public static double INTEGRAL_CLAMP = 50;

    // Internal state
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Heading / encoder reference
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

    /** Preferred constructor: Pinpoint + fallback IMU. */
    public TurretController(DcMotor turretMotor, BNO055IMU imu, GoBildaPinpointDriver pinpoint, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
        captureReferences();
        resetPidState();
    }

    /** Backward-compatible constructor: IMU only. */
    public TurretController(DcMotor turretMotor, BNO055IMU imu, Telemetry telemetry) {
        this(turretMotor, imu, null, telemetry);
    }

    /** Capture the current heading and turret encoder as the reference points. */
    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;
        turretEncoderReference = turretMotor.getCurrentPosition();
    }

    /** Reset PID internal state. */
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

        double ticksPerRad = ((maxPosCfg - minPosCfg) / (2.0 * Math.PI)) * ticksPerRadScaleCfg;

        if (manualNow) {
            applyManualPower(manualPower, minPosCfg, maxPosCfg);
            turretIntegral = 0.0;
            lastErrorTicks = 0;
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            manualActiveLast = true;
            publishTelemetry();
            return;
        }

        if (manualActiveLast && !manualNow) {
            captureReferences();
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        double currentHeadingRad = getHeadingRadians();
        double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);

        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }
        lastHeadingRad = currentHeadingRad;

        double desiredTicksDouble = turretEncoderReference - headingDelta * ticksPerRad;
        int desiredTicks = (int) Math.round(desiredTicksDouble);
        if (desiredTicks > maxPosCfg) desiredTicks = maxPosCfg;
        if (desiredTicks < minPosCfg) desiredTicks = minPosCfg;

        int currentTicks = turretMotor.getCurrentPosition();
        int errorTicks = desiredTicks - currentTicks;

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

        double ff = -angularVel * ffGainCfg;

        double cmdPower = pidOut + ff;
        if (Math.abs(errorTicks) <= deadbandCfg) cmdPower = 0.0;
        if (cmdPower > maxPowerCfg) cmdPower = maxPowerCfg;
        if (cmdPower < -maxPowerCfg) cmdPower = -maxPowerCfg;

        double applied = powerSmoothCfg * lastAppliedPower + (1.0 - powerSmoothCfg) * cmdPower;

        if ((currentTicks >= maxPosCfg && applied > 0.0) || (currentTicks <= minPosCfg && applied < 0.0)) {
            applied = 0.0;
        }

        turretMotor.setPower(applied);

        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;

        lastDesiredTicks = desiredTicks;
        lastErrorReported = errorTicks;
        lastPidOut = pidOut;
        lastFf = ff;

        publishTelemetry();
    }

    private void applyManualPower(double manualPower, int minPosCfg, int maxPosCfg) {
        int currentTicks = turretMotor.getCurrentPosition();

        double requested = manualPower;
        if ((currentTicks >= maxPosCfg && requested > 0.0) || (currentTicks <= minPosCfg && requested < 0.0)) {
            requested = 0.0;
        }
        if (requested > 1.0) requested = 1.0;
        if (requested < -1.0) requested = -1.0;

        turretMotor.setPower(requested);
        lastDesiredTicks = turretEncoderReference;
        lastErrorReported = lastDesiredTicks - currentTicks;
        lastPidOut = 0.0;
        lastFf = 0.0;
    }

    /**
     * Return current heading (Z) in radians.
     * Prefer Pinpoint (negated to match prior BNO convention); fall back to BNO IMU.
     */
    private double getHeadingRadians() {
        if (pinpoint != null) {
            try {
                return -pinpoint.getHeading(AngleUnit.RADIANS);
            } catch (Exception ignored) { }
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

    private void publishTelemetry() {
        if (telemetry == null) return;
//        telemetry.addData("turret.desired", lastDesiredTicks);
//        telemetry.addData("turret.error", lastErrorReported);
//        telemetry.addData("turret.pid", String.format("%.4f", lastPidOut));
//        telemetry.addData("turret.ff", String.format("%.4f", lastFf));
//        telemetry.addData("turret.applied", String.format("%.4f", lastAppliedPower));
    }
}