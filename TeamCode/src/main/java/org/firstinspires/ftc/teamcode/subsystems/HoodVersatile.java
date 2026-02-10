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
    private double lastDistance = 0.0;
    private double lastValidDistance = 0.0;
    private boolean isInitialized = false;

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

        // Calculate distances
        this.closeDistance = calculateDistanceBlue(closePose);
        this.farDistance = calculateDistanceBlue(farPose);

        // Initialize with close distance
        this.lastDistance = closeDistance;
        this.lastValidDistance = closeDistance;
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

    /**
     * Check if a distance value is reasonable
     */
    private boolean isReasonableDistance(double distance) {
        return distance >= 10.0 && distance <= 200.0;
    }

    /**
     * Check if a pose is valid
     */
    private boolean isValidPose(Pose pose) {
        if (pose == null) {
            return false;
        }

        double x = pose.getX();
        double y = pose.getY();

        // Reject origin
        if (Math.abs(x) < 1.0 && Math.abs(y) < 1.0) {
            return false;
        }

        // Reject out of field bounds
        if (x < -5 || x > 150 || y < -5 || y > 150) {
            return false;
        }

        return true;
    }

    public double getDistanceToGoal(Pose pose) {
        return calculateDistance(pose);
    }

    public double computeBasePosition(Pose robotPose) {
        // Validate pose
        if (!isValidPose(robotPose)) {
            return lastTargetPos;
        }

        double newDistance = calculateDistance(robotPose);

        // Validate distance
        if (!isReasonableDistance(newDistance)) {
            return lastTargetPos;
        }

        // Check for sudden jumps to detect sensor errors
        // BUT allow the first real initialization to set any valid distance
        if (isInitialized && Math.abs(newDistance - lastValidDistance) > CalibrationPoints.MAX_DISTANCE_JUMP) {
            return lastTargetPos;
        }

        // Accept this distance
        lastDistance = newDistance;
        lastValidDistance = newDistance;
        isInitialized = true;

        // Calculate position
        lastTargetPos = computePositionFromDistance(lastDistance);

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

    private double clamp(double value) {
        return Math.max(minPos, Math.min(maxPos, value));
    }
}