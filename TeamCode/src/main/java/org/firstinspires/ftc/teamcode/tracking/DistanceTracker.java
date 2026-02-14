package org.firstinspires.ftc.teamcode.tracking;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelVersatile;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.subsystems.HoodVersatile;

/**
 * Central orchestrator: gets position, validates it, calculates distance/RPM/hood.
 *
 * Combines pose tracking with FlywheelVersatile (auto RPM) and HoodVersatile (auto hood)
 * so the teleop only needs one update call to get everything computed.
 *
 * Pose validation:
 *   - Rejects null, origin, and out-of-field poses
 *   - Rejects sudden jumps (sensor errors)
 *   - Requires multiple consecutive valid poses before accepting
 */
public class DistanceTracker {

    private final Pose startPose;
    private final boolean isRedAlliance;

    // Subsystems for auto RPM and auto hood
    private final FlywheelVersatile flywheelVersatile;
    private final HoodVersatile hoodVersatile;       // update() internally sets hood position
    private final FlywheelController flywheelController;

    private Pose currentPose;
    private Pose lastValidPose;
    private int validPoseCount = 0;
    private static final int REQUIRED_VALID_POSES = 5;

    // Last computed values
    private double lastTargetRpm = 0.0;
    private double lastTargetHood = 0.0;
    private double lastDistance = 0.0;

    /**
     * @param startPose         Robot starting pose for this alliance
     * @param isRedAlliance     true for red, false for blue
     * @param flywheelVersatile Computes target RPM from distance (null to disable auto RPM)
     * @param hoodVersatile     Computes target hood from distance (null to disable auto hood)
     * @param flywheelController FlywheelController to set target RPM on (null to skip)
     */
    public DistanceTracker(Pose startPose, boolean isRedAlliance,
                           FlywheelVersatile flywheelVersatile,
                           HoodVersatile hoodVersatile,
                           FlywheelController flywheelController) {
        this.startPose = startPose;
        this.isRedAlliance = isRedAlliance;
        this.flywheelVersatile = flywheelVersatile;
        this.hoodVersatile = hoodVersatile;
        this.flywheelController = flywheelController;
        this.currentPose = startPose;
        this.lastValidPose = startPose;

        // Compute initial values from start pose
        if (flywheelVersatile != null) {
            lastTargetRpm = flywheelVersatile.getFinalTargetRPM(startPose);
        }
        if (hoodVersatile != null) {
            lastTargetHood = hoodVersatile.getFinalTargetPosition(startPose);
        }
        lastDistance = CalibrationPoints.distanceToGoal(startPose, isRedAlliance);
    }

    /**
     * Update pose from follower with validation, then compute RPM and hood.
     * Call this each loop iteration after follower.update().
     *
     * @param autoFlywheel if true, compute and apply auto RPM
     * @param autoHood     if true, compute and apply auto hood
     */
    public void update(Follower follower, boolean autoFlywheel, boolean autoHood) {
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

        // Compute distance
        lastDistance = CalibrationPoints.distanceToGoal(currentPose, isRedAlliance);

        // Compute and apply auto RPM
        if (autoFlywheel && flywheelVersatile != null) {
            lastTargetRpm = flywheelVersatile.getFinalTargetRPM(currentPose);
            if (flywheelController != null) {
                flywheelController.setTargetRPM(lastTargetRpm);
            }
        }

        // Compute and apply auto hood
        if (autoHood && hoodVersatile != null) {
            hoodVersatile.update(currentPose);
            lastTargetHood = hoodVersatile.getLastTargetPos();
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

    // ==================== GETTERS ====================

    public Pose getCurrentPose() {
        return currentPose;
    }

    public double getDistanceToGoal() {
        return lastDistance;
    }

    public double getLastTargetRpm() {
        return lastTargetRpm;
    }

    public double getLastTargetHood() {
        return lastTargetHood;
    }

    public int getValidPoseCount() {
        return validPoseCount;
    }

    public void resetValidPoseCount() {
        validPoseCount = 0;
    }

    public FlywheelVersatile getFlywheelVersatile() {
        return flywheelVersatile;
    }

    public HoodVersatile getHoodVersatile() {
        return hoodVersatile;
    }

    // ==================== POSE VALIDATION ====================

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
