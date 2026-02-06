package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

/**
 * Computes target hood position from robot pose relative to a goal.
 * Uses linear interpolation between two calibration points:
 * - Close pose (e.g., 20, 122) -> minPos (hood low)
 * - Far pose (e.g., 72, 12) -> maxPos (hood high)
 *
 * As the robot moves farther from the goal along this diagonal,
 * the hood rises proportionally.
 */
public class HoodVersatile {

    private final HoodController hoodController;
    private final Pose goalPose;      // The goal/target location
    private final Pose closePose;     // Calibration: closest shooting point
    private final Pose farPose;       // Calibration: farthest shooting point
    private final double minPos;      // Hood position when close
    private final double maxPos;      // Hood position when far

    private double slope = 0.0;
    private double intercept = 0.0;
    private double trimPos = 0.0;     // Driver trim adjustment
    private double lastTargetPos = 0.0;
    private double lastDistance = 0.0;

    /**
     * @param hoodController The underlying hood controller
     * @param goalPose       The goal location (for distance reference)
     * @param closePose      Pose where hood should be at minimum (e.g., 20, 122)
     * @param farPose        Pose where hood should be at maximum (e.g., 72, 12)
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
        this.goalPose = goalPose;
        this.closePose = closePose;
        this.farPose = farPose;
        this.minPos = minPos;
        this.maxPos = maxPos;
        this.lastTargetPos = minPos;
        computeRegression();
    }

    /**
     * Computes linear regression: hood position as a function of distance to goal.
     * Two-point form: we know (closeDistance -> minPos) and (farDistance -> maxPos).
     */
    private void computeRegression() {
        double closeDist = distanceToGoal(closePose);
        double farDist = distanceToGoal(farPose);

        if (Math.abs(farDist - closeDist) < 1e-6) {
            // Edge case: same distance, no slope
            slope = 0.0;
            intercept = minPos;
            return;
        }

        // Linear regression: pos = slope * distance + intercept
        // Two points: (closeDist, minPos) and (farDist, maxPos)
        slope = (maxPos - minPos) / (farDist - closeDist);
        intercept = minPos - slope * closeDist;
    }

    private double distanceToGoal(Pose pose) {
        double dx = pose.getX() - goalPose.getX();
        double dy = pose.getY() - goalPose.getY();
        return Math.hypot(dx, dy);
    }

    public double getDistanceToGoal(Pose pose) {
        return distanceToGoal(pose);
    }

    /**
     * Computes the base hood position based on current robot pose.
     */
    public double computeBasePosition(Pose robotPose) {
        if (robotPose == null) {
            return lastTargetPos;
        }
        lastDistance = distanceToGoal(robotPose);
        double raw = slope * lastDistance + intercept;
        lastTargetPos = clamp(raw);
        return lastTargetPos;
    }

    /**
     * Gets the final target position including driver trim.
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
        double targetPos = getFinalTargetPosition(robotPose);
        hoodController.setRightPosition(targetPos);
    }

    /**
     * Adjusts the driver trim for fine-tuning.
     */
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