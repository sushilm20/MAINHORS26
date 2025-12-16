package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * PID-based turret controller that auto-aims toward a field goal point using robot pose.
 * Uses the same gains/limits as your prior controller, but target comes from goal bearing.
 */
public class TurretGoalAimer {

    // Hardware
    private final DcMotor turretMotor;
    private final BNO055IMU imu;
    private final Telemetry telemetry; // optional

    // Limits
    public static final int TURRET_MIN_POS = -600;
    public static final int TURRET_MAX_POS = 600;

    // Ticks mapping
    private static final double BASE_TICKS_PER_RADIAN = TURRET_MAX_POS / Math.PI;
    private static final double TICKS_PER_RADIAN_SCALE = 2;
    private static final double TICKS_PER_RADIAN = BASE_TICKS_PER_RADIAN * TICKS_PER_RADIAN_SCALE;

    // Gains
    private static final double TURRET_KP = 1.3;
    private static final double TURRET_KI = 0.09;
    private static final double TURRET_KD = 0.3;
    private static final double TURRET_MAX_POWER = 1.0;

    // FF & smoothing
    private static final double FF_GAIN = 0.030;
    private static final double POWER_SMOOTH_ALPHA = 0.96;
    private static final double DERIV_FILTER_ALPHA = 0.40;

    // Deadband & anti-windup
    private static final int SMALL_DEADBAND_TICKS = 4;
    private static final double INTEGRAL_CLAMP = 200.0;

    // State
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    private double headingReferenceRad = 0.0;
    private double lastHeadingRad = 0.0;
    private int turretEncoderReference = 0;
    private boolean manualActiveLast = false;

    // Telemetry values
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;

    public TurretGoalAimer(DcMotor turretMotor, BNO055IMU imu, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.telemetry = telemetry;
        captureReferences();
        resetPidState();
    }

    /** Capture IMU heading and turret encoder as reference. */
    public void captureReferences() {
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;
        turretEncoderReference = turretMotor.getCurrentPosition();
    }

    /** Reset PID state. */
    public void resetPidState() {
        turretIntegral = 0.0;
        lastErrorTicks = 0;
        lastTimeMs = System.currentTimeMillis();
        lastAppliedPower = 0.0;
        lastDerivative = 0.0;
        manualActiveLast = false;
    }

    /** Legacy overload (falls back to heading-hold behavior). */
    public void update(boolean manualNow, double manualPower) {
        update(manualNow, manualPower, null, null);
    }

    /**
     * Main update: aims at goalPose using robotPose.
     * @param robotPose robot pose (PedroPathing Pose)
     * @param goalPose  field goal to aim at
     */
    public void update(boolean manualNow, double manualPower, Pose robotPose, Pose goalPose) {
        long nowMs = System.currentTimeMillis();

        if (manualNow) {
            applyManualPower(manualPower);
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
        double targetAngleRad;

        if (robotPose != null && goalPose != null) {
            double dx = goalPose.getX() - robotPose.getX();
            double dy = goalPose.getY() - robotPose.getY();
            double goalBearing = Math.atan2(dy, dx); // field-centric
            double robotHeading = currentHeadingRad;
            double relativeAngle = normalizeAngle(goalBearing - robotHeading);
            targetAngleRad = relativeAngle;
        } else {
            double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);
            targetAngleRad = -headingDelta; // fallback
        }

        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }
        lastHeadingRad = currentHeadingRad;

        double desiredTicksDouble = turretEncoderReference - targetAngleRad * TICKS_PER_RADIAN;
        int desiredTicks = (int)Math.round(desiredTicksDouble);
        if (desiredTicks > TURRET_MAX_POS) desiredTicks = TURRET_MAX_POS;
        if (desiredTicks < TURRET_MIN_POS) desiredTicks = TURRET_MIN_POS;

        int currentTicks = turretMotor.getCurrentPosition();
        int errorTicks = desiredTicks - currentTicks;

        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        if (Math.abs(errorTicks) > SMALL_DEADBAND_TICKS) {
            if (lastErrorTicks != 0 && ((errorTicks > 0 && lastErrorTicks < 0) || (errorTicks < 0 && lastErrorTicks > 0))) {
                turretIntegral *= 0.5;
            }
            turretIntegral += errorTicks * dt;
        } else {
            turretIntegral *= 0.90;
        }

        if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
        if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = DERIV_FILTER_ALPHA * rawDerivative + (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

        double pidOut = TURRET_KP * errorTicks + TURRET_KI * turretIntegral + TURRET_KD * derivativeFiltered;
        double ff = -angularVel * FF_GAIN;

        double cmdPower = pidOut + ff;
        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) cmdPower = 0.0;
        if (cmdPower > TURRET_MAX_POWER) cmdPower = TURRET_MAX_POWER;
        if (cmdPower < -TURRET_MAX_POWER) cmdPower = -TURRET_MAX_POWER;

        double applied = POWER_SMOOTH_ALPHA * lastAppliedPower + (1.0 - POWER_SMOOTH_ALPHA) * cmdPower;

        if ((currentTicks >= TURRET_MAX_POS && applied > 0.0) || (currentTicks <= TURRET_MIN_POS && applied < 0.0)) {
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

    private void applyManualPower(double manualPower) {
        int currentTicks = turretMotor.getCurrentPosition();
        double requested = manualPower;
        if ((currentTicks >= TURRET_MAX_POS && requested > 0.0) || (currentTicks <= TURRET_MIN_POS && requested < 0.0)) {
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

    // Telemetry getters
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
