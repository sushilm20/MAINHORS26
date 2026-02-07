package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.tracking.CalibrationPoints;

/**
 * Computes target hood position from robot pose relative to goal.
 *
 * CLOSE to goal = hood at MIN (0.12)
 * FAR from goal = hood at MAX (0.48)
 *
 * Supports auto-mirroring for red alliance.
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
        this.lastTargetPos = minPos;  // Start at minimum

        // Calculate distances for calibration poses
        this.closeDistance = calculateDistanceBlue(closePose);
        this.farDistance = calculateDistanceBlue(farPose);

        // Compute linear regression
        computeRegression();

        // Set hood to initial position (minimum)
        if (hoodController != null) {
            hoodController.setRightPosition(minPos);
        }
    }

    public void setRedAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
    }

    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**
     * Linear regression: position = slope * distance + intercept
     *
     * At closeDistance -> minPos
     * At farDistance -> maxPos
     */
    private void computeRegression() {
        double distanceRange = farDistance - closeDistance;

        if (Math.abs(distanceRange) < 1e-6) {
            slope = 0.0;
            intercept = minPos;
            return;
        }

        // slope = (maxPos - minPos) / (farDistance - closeDistance)
        slope = (maxPos - minPos) / distanceRange;

        // intercept = minPos - slope * closeDistance
        intercept = minPos - slope * closeDistance;
    }

    /**
     * Distance to goal using BLUE coordinates
     */
    private double calculateDistanceBlue(Pose pose) {
        double dx = pose.getX() - blueGoalPose.getX();
        double dy = pose.getY() - blueGoalPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Distance to goal with auto-mirroring for red alliance
     */
    private double calculateDistance(Pose robotPose) {
        Pose effectivePose = robotPose;

        if (isRedAlliance && robotPose != null) {
            effectivePose = robotPose.mirror(MIRROR_AXIS);
        }

        return calculateDistanceBlue(effectivePose);
    }

    public double getDistanceToGoal(Pose pose) {
        return calculateDistance(pose);
    }

    /**
     * Compute hood position for given pose
     */
    public double computeBasePosition(Pose robotPose) {
        if (robotPose == null) {
            return lastTargetPos;
        }

        lastDistance = calculateDistance(robotPose);

        // Linear interpolation: pos = slope * distance + intercept
        double raw = slope * lastDistance + intercept;

        // Clamp to valid range
        lastTargetPos = clamp(raw);

        return lastTargetPos;
    }

    /**
     * Get final position including trim adjustment
     */
    public double getFinalTargetPosition(Pose robotPose) {
        double base = computeBasePosition(robotPose);
        return clamp(base + trimPos);
    }

    /**
     * Update hood servo to match current pose
     */
    public void update(Pose robotPose) {
        if (robotPose == null || hoodController == null) {
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

    private double clamp(double value) {
        return Math.max(minPos, Math.min(maxPos, value));
    }
}