package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.tracking.CalibrationPoints;

/**
 * Computes target hood position from robot pose relative to a goal.
 * Supports BLUE and RED alliance - auto-mirrors poses for red!
 */
public class HoodVersatile {

    private static final double MIRROR_AXIS = CalibrationPoints.MIRROR_AXIS;

    private final HoodController hoodController;
    private final Pose blueGoalPose;
    private final double closeDistance;  // Distance at which hood is at min
    private final double farDistance;    // Distance at which hood is at max
    private final double minPos;
    private final double maxPos;

    private boolean isRedAlliance = false;
    private boolean enabled = true;  // Can disable auto updates

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
        this.minPos = minPos;
        this.maxPos = maxPos;
        this.lastTargetPos = minPos;

        // Pre-compute distances for calibration poses
        this.closeDistance = distanceToGoalBlue(closePose);
        this.farDistance = distanceToGoalBlue(farPose);

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
     * Enable or disable automatic hood updates
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Computes linear regression: hood position as a function of distance.
     *
     * At closeDistance -> minPos
     * At farDistance -> maxPos
     */
    private void computeRegression() {
        if (Math.abs(farDistance - closeDistance) < 1e-6) {
            // Edge case: same distance, no slope
            slope = 0.0;
            intercept = minPos;
            return;
        }

        // Linear equation: pos = slope * distance + intercept
        // Two points: (closeDistance, minPos) and (farDistance, maxPos)
        slope = (maxPos - minPos) / (farDistance - closeDistance);
        intercept = minPos - slope * closeDistance;
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

    /**
     * Compute the base hood position (before trim) for a given pose
     */
    public double computeBasePosition(Pose robotPose) {
        if (robotPose == null) {
            return lastTargetPos;
        }

        lastDistance = distanceToGoal(robotPose);

        // Calculate raw position from linear regression
        double raw = slope * lastDistance + intercept;

        // Clamp to valid range
        lastTargetPos = clamp(raw);

        return lastTargetPos;
    }

    /**
     * Get the final target position including driver trim
     */
    public double getFinalTargetPosition(Pose robotPose) {
        double base = computeBasePosition(robotPose);
        return clamp(base + trimPos);
    }

    /**
     * Updates the hood controller with the pose-based position.
     * Call this each loop iteration.
     */
    public void update(Pose robotPose) {
        if (!enabled) {
            return;
        }

        double targetPos = getFinalTargetPosition(robotPose);
        hoodController.setRightPosition(targetPos);
    }

    /**
     * Force update even if disabled (for testing)
     */
    public void forceUpdate(Pose robotPose) {
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