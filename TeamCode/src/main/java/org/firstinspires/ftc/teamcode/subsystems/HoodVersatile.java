package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.tracking.CalibrationPoints;

/**
 * Computes target hood position from robot pose relative to goal.
 * CLOSE to goal = hood at MIN, FAR from goal = hood at MAX
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

    private double slope = 0.0;
    private double intercept = 0.0;
    private double trimPos = 0.0;
    private double lastTargetPos;
    private double lastDistance;
    private double lastValidDistance;

    // Debug tracking
    private int updateCount = 0;
    private String lastRejectReason = "";

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

        // Calculate distances for calibration poses
        this.closeDistance = calculateDistanceBlue(closePose);
        this.farDistance = calculateDistanceBlue(farPose);

        // Initialize to START_POSE values
        this.lastDistance = CalibrationPoints.START_DISTANCE;
        this.lastValidDistance = CalibrationPoints.START_DISTANCE;
        this.lastTargetPos = minPos;

        computeRegression();
    }

    public void setRedAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
    }

    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    private void computeRegression() {
        double distanceRange = farDistance - closeDistance;

        if (Math.abs(distanceRange) < 1e-6) {
            slope = 0.0;
            intercept = minPos;
            return;
        }

        slope = (maxPos - minPos) / distanceRange;
        intercept = minPos - slope * closeDistance;
    }

    private double computePositionFromDistance(double distance) {
        double raw = slope * distance + intercept;
        return clamp(raw);
    }

    private double calculateDistanceBlue(Pose pose) {
        if (pose == null) {
            return lastValidDistance;
        }
        double dx = pose.getX() - blueGoalPose.getX();
        double dy = pose.getY() - blueGoalPose.getY();
        return Math.hypot(dx, dy);
    }

    private double calculateDistance(Pose robotPose) {
        if (robotPose == null) {
            return lastValidDistance;
        }

        Pose effectivePose = robotPose;

        if (isRedAlliance) {
            effectivePose = robotPose.mirror(MIRROR_AXIS);
        }

        return calculateDistanceBlue(effectivePose);
    }

    public double getDistanceToGoal(Pose pose) {
        return calculateDistance(pose);
    }

    /**
     * Compute base hood position - SIMPLIFIED VERSION
     */
    public double computeBasePosition(Pose robotPose) {
        updateCount++;
        lastRejectReason = "";

        // Null check
        if (robotPose == null) {
            lastRejectReason = "null pose";
            return lastTargetPos;
        }

        double x = robotPose.getX();
        double y = robotPose.getY();

        // Only reject obvious bad poses (at origin)
        if (Math.abs(x) < 0.5 && Math.abs(y) < 0.5) {
            lastRejectReason = "origin pose";
            return lastTargetPos;
        }

        // Calculate distance
        double newDistance = calculateDistance(robotPose);

        // Basic sanity check
        if (newDistance < 5.0 || newDistance > 250.0) {
            lastRejectReason = "distance out of range: " + newDistance;
            return lastTargetPos;
        }

        // ACCEPT the distance
        lastDistance = newDistance;
        lastValidDistance = newDistance;

        // Calculate hood position
        lastTargetPos = computePositionFromDistance(newDistance);

        return lastTargetPos;
    }

    public double getFinalTargetPosition(Pose robotPose) {
        double base = computeBasePosition(robotPose);
        return clamp(base + trimPos);
    }

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

    public double getLastValidDistance() {
        return lastValidDistance;
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

    public int getUpdateCount() {
        return updateCount;
    }

    public String getLastRejectReason() {
        return lastRejectReason;
    }

    private double clamp(double value) {
        return Math.max(minPos, Math.min(maxPos, value));
    }
}