package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.tracking.CalibrationPoints;

/**
 * Computes target hood position from robot pose.
 * CLOSE to goal = hood at MIN (0.12)
 * FAR from goal = hood at MAX (0.48)
 *
 * Uses linear regression between two calibration points.
 */
public class HoodVersatile {

    private static final double MIRROR_AXIS = CalibrationPoints.MIRROR_AXIS;

    private final HoodController hoodController;
    private final Pose blueGoalPose;
    private final double closeDistance;
    private final double farDistance;
    private final double minPos;
    private final double maxPos;

    private boolean isRedAlliance = false;

    // Linear regression: pos = slope * distance + intercept
    private double slope = 0.0;
    private double intercept = 0.0;

    private double trimPos = 0.0;
    private double lastTargetPos;
    private double lastDistance;

    public HoodVersatile(HoodController hoodController,
                         Pose goalPose,
                         Pose closePose,
                         Pose farPose,
                         double minPos,
                         double maxPos) {
        this.hoodController = hoodController;
        this.blueGoalPose = goalPose;
        this.minPos = minPos;
        this.maxPos = maxPos;

        // Calculate distances for the two calibration poses
        this.closeDistance = calcDistBlue(closePose);  // ~30 units
        this.farDistance = calcDistBlue(farPose);      // ~150 units

        // Initialize with start values
        this.lastDistance = CalibrationPoints.START_DISTANCE;
        this.lastTargetPos = minPos;

        // Compute linear regression
        computeRegression();
    }

    public void setRedAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
    }

    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**
     * Compute linear regression coefficients
     * At closeDistance -> minPos
     * At farDistance -> maxPos
     */
    private void computeRegression() {
        double dRange = farDistance - closeDistance;

        if (Math.abs(dRange) < 0.001) {
            slope = 0.0;
            intercept = minPos;
            return;
        }

        // slope = (maxPos - minPos) / (farDist - closeDist)
        slope = (maxPos - minPos) / dRange;

        // intercept = minPos - slope * closeDist
        intercept = minPos - slope * closeDistance;
    }

    /**
     * Calculate hood position from distance
     */
    private double posFromDist(double dist) {
        double raw = slope * dist + intercept;
        return clamp(raw);
    }

    /**
     * Calculate distance from pose to BLUE goal
     */
    private double calcDistBlue(Pose pose) {
        if (pose == null) return lastDistance;
        double dx = pose.getX() - blueGoalPose.getX();
        double dy = pose.getY() - blueGoalPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Calculate distance with alliance mirroring
     */
    private double calcDist(Pose robotPose) {
        if (robotPose == null) return lastDistance;

        Pose effectivePose = robotPose;
        if (isRedAlliance) {
            effectivePose = robotPose.mirror(MIRROR_AXIS);
        }

        return calcDistBlue(effectivePose);
    }

    public double getDistanceToGoal(Pose pose) {
        return calcDist(pose);
    }

    /**
     * Main computation - SIMPLE and DIRECT
     */
    public double computeBasePosition(Pose robotPose) {
        // Handle null
        if (robotPose == null) {
            return lastTargetPos;
        }

        // Get X, Y
        double x = robotPose.getX();
        double y = robotPose.getY();

        // ONLY reject if literally at origin (0,0)
        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
            return lastTargetPos;
        }

        // Calculate distance
        double dist = calcDist(robotPose);

        // Store for telemetry
        lastDistance = dist;

        // Calculate hood position from linear regression
        lastTargetPos = posFromDist(dist);

        return lastTargetPos;
    }

    public double getFinalTargetPosition(Pose robotPose) {
        double base = computeBasePosition(robotPose);
        return clamp(base + trimPos);
    }

    /**
     * Update the hood servo
     */
    public void update(Pose robotPose) {
        if (hoodController == null) {
            return;
        }

        double targetPos = getFinalTargetPosition(robotPose);
        hoodController.setRightPosition(targetPos);
    }

    public void adjustTrim(double delta) {
        trimPos += delta;
    }

    public void resetTrim() {
        trimPos = 0.0;
    }

    public double getTrimPos() {
        return trimPos;
    }

    public double getLastTargetPos() {
        return lastTargetPos;
    }

    public double getLastDistance() {
        return lastDistance;
    }

    public double getMinPos() {
        return minPos;
    }

    public double getMaxPos() {
        return maxPos;
    }

    public double getSlope() {
        return slope;
    }

    public double getIntercept() {
        return intercept;
    }

    public double getCloseDistance() {
        return closeDistance;
    }

    public double getFarDistance() {
        return farDistance;
    }

    private double clamp(double val) {
        return Math.max(minPos, Math.min(maxPos, val));
    }
}