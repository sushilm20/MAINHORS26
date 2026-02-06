package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

/**
 * Computes target hood position from robot pose relative to a goal.
 * Supports BLUE and RED alliance - auto-mirrors poses for red!
 */
public class HoodVersatile {

    private static final double MIRROR_AXIS = 146.0;

    private final HoodController hoodController;
    private final Pose blueGoalPose;
    private final Pose blueClosePose;
    private final Pose blueFarPose;
    private final double minPos;
    private final double maxPos;

    private boolean isRedAlliance = false;

    private double slope = 0.0;
    private double intercept = 0.0;
    private double trimPos = 0.0;
    private double lastTargetPos = 0.0;
    private double lastDistance = 0.0;

    /**
     * @param hoodController The underlying hood controller
     * @param goalPose       The BLUE goal location
     * @param closePose      BLUE pose where hood should be at minimum
     * @param farPose        BLUE pose where hood should be at maximum
     * @param minPos         Hood servo position when close
     * @param maxPos         Hood servo position when far
     */
    public HoodVersatile(HoodController hoodController,
                         Pose goalPose,
                         Pose closePose,
                         Pose farPose,
                         double minPos,
                         double maxPos) {
        this.hoodController = hoodController;
        this.blueGoalPose = goalPose;
        this.blueClosePose = closePose;
        this.blueFarPose = farPose;
        this.minPos = minPos;
        this.maxPos = maxPos;
        this.lastTargetPos = minPos;
        computeRegression();
    }

    /**
     * Set red alliance mode - auto mirrors all poses!
     */
    public void setRedAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
    }

    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**
     * Computes linear regression using BLUE coordinates.
     */
    private void computeRegression() {
        double closeDist = distanceToGoalBlue(blueClosePose);
        double farDist = distanceToGoalBlue(blueFarPose);

        if (Math.abs(farDist - closeDist) < 1e-6) {
            slope = 0.0;
            intercept = minPos;
            return;
        }

        slope = (maxPos - minPos) / (farDist - closeDist);
        intercept = minPos - slope * closeDist;
    }

    /**
     * Distance using BLUE coordinates (for calibration)
     */
    private double distanceToGoalBlue(Pose pose) {
        double dx = pose.getX() - blueGoalPose.getX();
        double dy = pose.getY() - blueGoalPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Distance that auto-mirrors for red alliance
     */
    private double distanceToGoal(Pose robotPose) {
        Pose effectivePose = robotPose;

        // Mirror robot pose if red alliance
        if (isRedAlliance) {
            effectivePose = robotPose.mirror(MIRROR_AXIS);
        }

        // Always calculate to BLUE goal
        double dx = effectivePose.getX() - blueGoalPose.getX();
        double dy = effectivePose.getY() - blueGoalPose.getY();
        return Math.hypot(dx, dy);
    }

    public double getDistanceToGoal(Pose pose) {
        return distanceToGoal(pose);
    }

    public double computeBasePosition(Pose robotPose) {
        if (robotPose == null) {
            return lastTargetPos;
        }
        lastDistance = distanceToGoal(robotPose);
        double raw = slope * lastDistance + intercept;
        lastTargetPos = clamp(raw);
        return lastTargetPos;
    }

    public double getFinalTargetPosition(Pose robotPose) {
        double base = computeBasePosition(robotPose);
        return clamp(base + trimPos);
    }

    /**
     * Updates the hood controller with the pose-based position.
     */
    public void update(Pose robotPose) {
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

    private double clamp(double value) {
        return Math.max(minPos, Math.min(maxPos, value));
    }
}