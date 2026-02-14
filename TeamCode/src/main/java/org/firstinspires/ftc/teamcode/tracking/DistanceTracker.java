package org.firstinspires.ftc.teamcode.tracking;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Centralizes pose validation, distance-to-goal calculation,
 * and provides validated pose updates from a PedroPathing Follower.
 *
 * Pose validation:
 *   - Rejects null, origin, and out-of-field poses
 *   - Rejects sudden jumps (sensor errors)
 *   - Requires multiple consecutive valid poses before accepting
 */
public class DistanceTracker {

    private final Pose startPose;
    private final boolean isRedAlliance;

    private Pose currentPose;
    private Pose lastValidPose;
    private int validPoseCount = 0;
    private static final int REQUIRED_VALID_POSES = 5;

    public DistanceTracker(Pose startPose, boolean isRedAlliance) {
        this.startPose = startPose;
        this.isRedAlliance = isRedAlliance;
        this.currentPose = startPose;
        this.lastValidPose = startPose;
    }

    /**
     * Update pose from follower with validation.
     * Call this each loop iteration after follower.update().
     */
    public void update(Follower follower) {
        if (follower == null) return;

        Pose rawPose = follower.getPose();

        if (isValidPose(rawPose) && isNearExpectedPose(rawPose, lastValidPose)) {
            validPoseCount++;
            if (validPoseCount >= REQUIRED_VALID_POSES) {
                currentPose = rawPose;
                lastValidPose = rawPose;
            }
        } else {
            validPoseCount = 0;
        }
    }

    /**
     * Reset pose tracking to start position.
     * Also resets the follower's starting pose if provided.
     */
    public void resetToStart(Follower follower) {
        if (follower != null) {
            follower.setStartingPose(startPose);
        }
        currentPose = startPose;
        lastValidPose = startPose;
        validPoseCount = 0;
    }

    /**
     * Get distance from current validated pose to the goal.
     */
    public double getDistanceToGoal() {
        return CalibrationPoints.distanceToGoal(currentPose, isRedAlliance);
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public int getValidPoseCount() {
        return validPoseCount;
    }

    public void resetValidPoseCount() {
        validPoseCount = 0;
    }

    /**
     * Check if pose is valid (not null, not at origin, within field bounds).
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

    /**
     * Check if new pose is reasonably close to expected pose
     * (prevents sudden jumps from sensor errors).
     */
    private boolean isNearExpectedPose(Pose newPose, Pose expectedPose) {
        if (newPose == null || expectedPose == null) {
            return false;
        }

        double dx = newPose.getX() - expectedPose.getX();
        double dy = newPose.getY() - expectedPose.getY();
        double distance = Math.hypot(dx, dy);

        return distance < CalibrationPoints.MAX_POSE_MOVEMENT_PER_CYCLE;
    }
}
