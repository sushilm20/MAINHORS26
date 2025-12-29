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
 * TurretGoalAimer:
 * - Faces a single field goal point.
 * - Uses robot pose + IMU heading.
 * - Adds lateral-offset correction: if the robot is left/right of the goal line, apply a small heading offset.
 * - Manual override is respected.
 */
public class TurretGoalAimer {

    // Hardware
    private final DcMotor turretMotor;
    private final BNO055IMU imu;
    private final Telemetry telemetry; // optional

    // Target to aim at (field coords). Replace via setTargetPose as needed.
    private Pose targetPose = new Pose(14, 134, 0); // default example (blue goal)

    // Limits (aligned with TurretController)
    public static final int TURRET_MIN_POS = -900;
    public static final int TURRET_MAX_POS = 900;

    // Mapping (aligned with TurretController)
    private static final double TICKS_PER_RADIAN_SCALE = 1.0;
    private static final double TICKS_PER_RADIAN =
            ((TURRET_MAX_POS - TURRET_MIN_POS) / (2.0 * Math.PI)) * TICKS_PER_RADIAN_SCALE;

    // Gains (aligned with TurretController)
    private static final double TURRET_KP = 1.2;
    private static final double TURRET_KI = 0.09;
    private static final double TURRET_KD = 0.3;
    private static final double TURRET_MAX_POWER = 1.0;

    // FF & smoothing
    private static final double FF_GAIN = 0.04;
    private static final double POWER_SMOOTH_ALPHA = 0.96;
    private static final double DERIV_FILTER_ALPHA = 0.40;

    // Deadband & anti-windup
    private static final int SMALL_DEADBAND_TICKS = 1;
    private static final double INTEGRAL_CLAMP = 0.0;

    // Lateral offset tuning (applied to target angle)
    private static final double OFFSET_GAIN_RAD_PER_IN = 0.005;               // ~0.29 deg per inch
    private static final double OFFSET_CLAMP_RAD = Math.toRadians(25.0);      // clamp offset to +/-25 deg

    // State
    private double turretIntegral = 0.0;
    private int lastErrorTicks = 0;
    private long lastTimeMs = -1L;
    private double lastAppliedPower = 0.0;
    private double lastDerivative = 0.0;

    // Reference mapping between heading and encoder
    private double headingReferenceRad = 0.0;
    private double lastHeadingRad = 0.0;
    private int turretEncoderReference = 0;
    private boolean manualActiveLast = false;

    // Telemetry values
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;
    private double lastOffsetRad = 0.0;
    private double lastTargetAngleRad = 0.0;
    private double lastGoalBearingRad = 0.0;

    public TurretGoalAimer(DcMotor turretMotor, BNO055IMU imu, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.telemetry = telemetry;
        captureReferences();
        resetPidState();
    }

    /** Set the field target to aim at. */
    public void setTargetPose(Pose target) {
        if (target != null) this.targetPose = target;
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

    /** Manual-only overload. */
    public void update(boolean manualNow, double manualPower) {
        update(manualNow, manualPower, null);
    }

    /**
     * Main update: aim at targetPose using robotPose, with lateral offset correction.
     * @param robotPose robot pose (PedroPathing Pose). If null, fall back to heading-hold.
     */
    public void update(boolean manualNow, double manualPower, Pose robotPose) {
        long nowMs = System.currentTimeMillis();

        // Manual override
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

        // Transition from manual -> auto
        if (manualActiveLast && !manualNow) {
            captureReferences();
            lastTimeMs = nowMs;
            lastDerivative = 0.0;
            lastAppliedPower = 0.0;
        }
        manualActiveLast = false;

        // Determine target angle relative to robot heading
        double currentHeadingRad = getHeadingRadians();
        double targetAngleRad;
        double goalBearing = 0.0;
        double offsetRad = 0.0;

        if (robotPose != null && targetPose != null) {
            double dx = targetPose.getX() - robotPose.getX();
            double dy = targetPose.getY() - robotPose.getY();
            goalBearing = Math.atan2(dy, dx);                // field-centric
            double robotHeading = currentHeadingRad;         // field-centric heading
            double relativeAngle = normalizeAngle(goalBearing - robotHeading);

            // Lateral error relative to goal line (using field Y as lateral reference to goal)
            // Positive error => robot is "left" (higher Y) of the goal line; negative => "right".
            double lateralErrorIn = (robotPose.getY() - targetPose.getY());
            offsetRad = OFFSET_GAIN_RAD_PER_IN * lateralErrorIn;
            if (offsetRad > OFFSET_CLAMP_RAD) offsetRad = OFFSET_CLAMP_RAD;
            if (offsetRad < -OFFSET_CLAMP_RAD) offsetRad = -OFFSET_CLAMP_RAD;

            targetAngleRad = normalizeAngle(relativeAngle + offsetRad);
        } else {
            // fallback: hold initial heading
            double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);
            targetAngleRad = -headingDelta;
        }

        lastGoalBearingRad = goalBearing;
        lastOffsetRad = offsetRad;
        lastTargetAngleRad = targetAngleRad;

        // Angular velocity for feedforward
        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }
        lastHeadingRad = currentHeadingRad;

        // Desired ticks from target angle
        double desiredTicksDouble = turretEncoderReference - targetAngleRad * TICKS_PER_RADIAN;
        int desiredTicks = (int)Math.round(desiredTicksDouble);
        if (desiredTicks > TURRET_MAX_POS) desiredTicks = TURRET_MAX_POS;
        if (desiredTicks < TURRET_MIN_POS) desiredTicks = TURRET_MIN_POS;

        int currentTicks = turretMotor.getCurrentPosition();
        int errorTicks = desiredTicks - currentTicks;

        // Timing
        long dtMs = (lastTimeMs < 0) ? 20 : Math.max(1, nowMs - lastTimeMs);
        double dt = dtMs / 1000.0;
        lastTimeMs = nowMs;

        // Integral with sign-change damping
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

        // PID + FF
        double pidOut = TURRET_KP * errorTicks + TURRET_KI * turretIntegral + TURRET_KD * derivativeFiltered;
        double ff = -angularVel * FF_GAIN;
        double cmdPower = pidOut + ff;

        // Deadband
        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) cmdPower = 0.0;

        // Clamp
        if (cmdPower > TURRET_MAX_POWER) cmdPower = TURRET_MAX_POWER;
        if (cmdPower < -TURRET_MAX_POWER) cmdPower = -TURRET_MAX_POWER;

        // Smooth
        double applied = POWER_SMOOTH_ALPHA * lastAppliedPower + (1.0 - POWER_SMOOTH_ALPHA) * cmdPower;

        // Hard-stop protection
        if ((currentTicks >= TURRET_MAX_POS && applied > 0.0) || (currentTicks <= TURRET_MIN_POS && applied < 0.0)) {
            applied = 0.0;
        }

        // Apply
        turretMotor.setPower(applied);

        // Update state
        lastErrorTicks = errorTicks;
        lastAppliedPower = applied;
        lastDerivative = derivativeFiltered;

        // Telemetry fields
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
        lastOffsetRad = 0.0;
        lastTargetAngleRad = 0.0;
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
    public double getLastOffsetDeg() { return Math.toDegrees(lastOffsetRad); }
    public double getLastGoalBearingDeg() { return Math.toDegrees(lastGoalBearingRad); }
    public double getLastTargetAngleDeg() { return Math.toDegrees(lastTargetAngleRad); }

    private void publishTelemetry() {
        if (telemetry == null) return;
//        telemetry.addData("turret.desired", lastDesiredTicks);
//        telemetry.addData("turret.error", lastErrorReported);
//        telemetry.addData("turret.pid", String.format("%.4f", lastPidOut));
//        telemetry.addData("turret.ff", String.format("%.4f", lastFf));
//        telemetry.addData("turret.applied", String.format("%.4f", lastAppliedPower));
//        telemetry.addData("turret.offset(deg)", String.format("%.2f", Math.toDegrees(lastOffsetRad)));
//        telemetry.addData("turret.targetAng(deg)", String.format("%.2f", Math.toDegrees(lastTargetAngleRad)));
//        telemetry.addData("turret.goalBearing(deg)", String.format("%.2f", Math.toDegrees(lastGoalBearingRad)));
    }
}