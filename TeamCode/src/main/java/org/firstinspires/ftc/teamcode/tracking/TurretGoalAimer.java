package org.firstinspires.ftc.teamcode.tracking;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
 * - Adds lateral-offset correction (side of line from start->goal).
 * - Manual override is respected.
 *
 * Pseudocode for point tracking with Pedro pathing + heading offsets (blue goal):
 * 1) Inputs: robotPose (Pedro Pose), targetPose (blue goal), current heading (from heading adjust system),
 *    turret encoder reference, turret encoder limits.
 * 2) If manual control requested -> apply manual power, exit.
 * 3) If robotPose missing -> heading-hold: desiredAngle = -(heading - headingReference).
 * 4) Else:
 *    a) bearing = atan2(target.y - robot.y, target.x - robot.x)   // field-centric
 *       (lineX = target.x - START.x, lineY = target.y - START.y; robotX = robot.x - START.x, robotY = robot.y - START.y)
 *    b) lateralError = (lineX * robotY - lineY * robotX) / distance(START, target)  // signed perpendicular distance (2D cross normalized by line length)
 *    c) offset = clamp(-lateralError * OFFSET_GAIN, +/- OFFSET_CLAMP) // keeps same point despite drift
 *    d) if aimReference not captured yet (first valid pose): aimReference = normalize(bearing - heading - offset)
 *    e) targetAngleRad = normalize(bearing - heading - offset - aimReference)
 *    f) desiredTicks = clamp(turretEncoderReference + targetAngleRad * TICKS_PER_RADIAN, min, max)
 *    g) error = desiredTicks - currentTicks
 *    h) pid = KP * error + KI * (integral of error) + KD * d(error) / dt
 *    i) ff = -angularVel * FF_GAIN                 // angularVel = robot yaw rate from IMU/pinpoint; oppose spin to hold lock
 *    j) power = clamp(pid + ff, +/- MAX_POWER) with deadband and soft smoothing; zero if at hard stops
 *    k) set turret motor power = power
 */
public class TurretGoalAimer {

    // Hardware
    private final DcMotor turretMotor;
    private final BNO055IMU imu;                     // fallback
    private final GoBildaPinpointDriver pinpoint;    // preferred
    private final Telemetry telemetry; // optional

    // Field references
    private static final Pose START_POSE = new Pose(20, 122, Math.toRadians(135));
    private Pose targetPose = new Pose(14, 134, Math.toRadians(135)); // default (blue goal)

    // Limits (aligned with TurretController)
    public static final int TURRET_MIN_POS = -1000;
    public static final int TURRET_MAX_POS = 1000;

    // Mapping (aligned with TurretController)
    private static final double TICKS_PER_RADIAN_SCALE = 0.9;
    private static final double TICKS_PER_RADIAN =
            ((TURRET_MAX_POS - TURRET_MIN_POS) / (2.0 * Math.PI)) * TICKS_PER_RADIAN_SCALE;

    // Gains (aligned with TurretController)
    private static final double TURRET_KP = 1.15;
    private static final double TURRET_KI = 0.0;
    private static final double TURRET_KD = 0.22;
    private static final double TURRET_MAX_POWER = 1.0;

    // FF & smoothing
    private static final double FF_GAIN = 5.0;
    private static final double POWER_SMOOTH_ALPHA = 0.935;
    private static final double DERIV_FILTER_ALPHA = 1.25;

    // Deadband & anti-windup
    private static final int SMALL_DEADBAND_TICKS = 14;
    private static final double INTEGRAL_CLAMP = 50.0;

    // Lateral offset tuning
    private static final double OFFSET_GAIN_RAD_PER_IN = 0.005;               // ~0.29 deg per inch
    private static final double OFFSET_CLAMP_RAD = Math.toRadians(25.0);      // clamp offset to +/-25 deg

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

    // Aim reference: ensures zero error at the starting pose
    private boolean aimReferenceCaptured = false;
    private double aimReferenceRad = 0.0; // (goalBearing - heading - offset) at start

    // Telemetry values
    private int lastDesiredTicks = 0;
    private int lastErrorReported = 0;
    private double lastPidOut = 0.0;
    private double lastFf = 0.0;
    private double lastOffsetRad = 0.0;
    private double lastTargetAngleRad = 0.0;
    private double lastGoalBearingRad = 0.0;

    public TurretGoalAimer(DcMotor turretMotor, BNO055IMU imu, GoBildaPinpointDriver pinpoint, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.imu = imu;
        this.pinpoint = pinpoint;
        this.telemetry = telemetry;
        captureReferences();
        resetPidState();
    }

    // Backward-compatible ctor (no Pinpoint)
    public TurretGoalAimer(DcMotor turretMotor, BNO055IMU imu, Telemetry telemetry) {
        this(turretMotor, imu, null, telemetry);
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
        // reset aim reference so it will re-capture next time we have a pose
        aimReferenceCaptured = false;
        aimReferenceRad = 0.0;
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
        double goalBearing = 0.0;
        double offsetRad = 0.0;

        if (robotPose != null && targetPose != null) {
            double dx = targetPose.getX() - robotPose.getX();
            double dy = targetPose.getY() - robotPose.getY();
            goalBearing = Math.atan2(dy, dx);                // field-centric

            // Line from START_POSE to targetPose
            double lineX = targetPose.getX() - START_POSE.getX();
            double lineY = targetPose.getY() - START_POSE.getY();
            double lineMag = Math.hypot(lineX, lineY);

            // Vector from START_POSE to robotPose
            double rX = robotPose.getX() - START_POSE.getX();
            double rY = robotPose.getY() - START_POSE.getY();

            // Signed perpendicular distance using 2D cross product (line x robot)
            double cross = lineX * rY - lineY * rX;
            double perpDist = (lineMag > 1e-6) ? cross / lineMag : 0.0; // inches, signed

            // Offset magnitude (sign flipped to apply in the opposite direction)
            offsetRad = -perpDist * OFFSET_GAIN_RAD_PER_IN;
            if (offsetRad > OFFSET_CLAMP_RAD) offsetRad = OFFSET_CLAMP_RAD;
            if (offsetRad < -OFFSET_CLAMP_RAD) offsetRad = -OFFSET_CLAMP_RAD;

            // Capture aim reference once (at the first valid pose) so turret doesn't jump at start
            if (!aimReferenceCaptured) {
                double startOffsetRad = offsetRad;
                double startGoalBearing = goalBearing;
                // Note: subtract offset here to match the flipped effect
                aimReferenceRad = normalizeAngle(startGoalBearing - currentHeadingRad - startOffsetRad);
                aimReferenceCaptured = true;
            }

            // Desired turret angle relative to the captured aim reference
            // FLIPPED EFFECT: subtract offsetRad (previously added)
            targetAngleRad = normalizeAngle((goalBearing - currentHeadingRad - offsetRad) - aimReferenceRad);

        } else {
            // Fallback: heading hold like TurretController
            double headingDelta = normalizeAngle(currentHeadingRad - headingReferenceRad);
            targetAngleRad = -headingDelta;
            aimReferenceCaptured = false; // force re-capture when pose returns
        }

        // Save telemetry values
        lastOffsetRad = offsetRad;
        lastGoalBearingRad = goalBearing;
        lastTargetAngleRad = targetAngleRad;

        // Compute angular velocity for feedforward
        double angularVel = 0.0;
        if (lastTimeMs > 0) {
            double dtHeading = Math.max(0.0001, (nowMs - lastTimeMs) / 1000.0);
            double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
            angularVel = headingDeltaSinceLast / dtHeading;
        }
        lastHeadingRad = currentHeadingRad;

        // Desired ticks from target angle (direction flipped: add instead of subtract)
        double desiredTicksDouble = turretEncoderReference + targetAngleRad * TICKS_PER_RADIAN;
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

        if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
        if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

        // Derivative (filtered)
        double rawDerivative = (errorTicks - lastErrorTicks) / Math.max(1e-4, dt);
        double derivativeFiltered = DERIV_FILTER_ALPHA * rawDerivative + (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

        // PID
        double pidOut = TURRET_KP * errorTicks + TURRET_KI * turretIntegral + TURRET_KD * derivativeFiltered;

        // Feedforward: oppose robot yaw rate
        double ff = -angularVel * FF_GAIN;

        double cmdPower = pidOut + ff;

        if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) {
            cmdPower = 0.0;
        }

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
        lastOffsetRad = 0.0;
        lastTargetAngleRad = 0.0;
        aimReferenceCaptured = false; // re-capture after manual
    }

    private double getHeadingRadians() {
        // Prefer Pinpoint; invert to match previous BNO sign convention. Fallback to BNO.
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
        // telemetry.addData("turret.desired", lastDesiredTicks);
        // telemetry.addData("turret.error", lastErrorReported);
        // telemetry.addData("turret.pid", String.format("%.4f", lastPidOut));
        // telemetry.addData("turret.ff", String.format("%.4f", lastFf));
        // telemetry.addData("turret.applied", String.format("%.4f", lastAppliedPower));
        // telemetry.addData("turret.offset(deg)", String.format("%.2f", Math.toDegrees(lastOffsetRad)));
        // telemetry.addData("turret.goalBearing(deg)", String.format("%.2f", Math.toDegrees(lastGoalBearingRad)));
        // telemetry.addData("turret.targetAngle(deg)", String.format("%.2f", Math.toDegrees(lastTargetAngleRad)));
    }
}