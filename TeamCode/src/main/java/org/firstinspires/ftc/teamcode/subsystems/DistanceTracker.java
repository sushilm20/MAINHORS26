package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.tracking.CalibrationPoints;

/**
 * Simple distance tracker that:
 * 1. Gets pose from follower
 * 2. Validates pose (rejects origin)
 * 3. Calculates distance to goal
 * 4. Returns RPM and hood values
 */
public class DistanceTracker {

    private final boolean isRedAlliance;
    private final double mirrorAxis;

    // Last known good values
    private double lastX;
    private double lastY;
    private double lastDistance;
    private double lastRpm;
    private double lastHood;

    // Debug info
    private boolean lastPoseWasValid = false;
    private String rejectReason = "";

    public DistanceTracker(boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;
        this.mirrorAxis = CalibrationPoints.MIRROR_AXIS;

        // Initialize to start position
        if (isRedAlliance) {
            Pose redStart = CalibrationPoints.BLUE_START_POSE.mirror(mirrorAxis);
            this.lastX = redStart.getX();
            this.lastY = redStart.getY();
        } else {
            this.lastX = CalibrationPoints.START_X;
            this.lastY = CalibrationPoints.START_Y;
        }

        // Calculate initial distance and values
        this.lastDistance = calculateDistance(lastX, lastY);
        this.lastRpm = CalibrationPoints.rpmFromDistance(lastDistance);
        this.lastHood = CalibrationPoints.hoodFromDistance(lastDistance);
    }

    /**
     * Update with new pose from follower.
     * Call this every loop.
     */
    public void update(Follower follower) {
        if (follower == null) {
            rejectReason = "follower null";
            lastPoseWasValid = false;
            return;
        }

        Pose pose = follower.getPose();
        if (pose == null) {
            rejectReason = "pose null";
            lastPoseWasValid = false;
            return;
        }

        double rawX = pose.getX();
        double rawY = pose.getY();

        // CHECK 1: Reject if at origin (0, 0)
        // This is the main problem - follower returns (0,0) sometimes
        if (Math.abs(rawX) < 1.0 && Math.abs(rawY) < 1.0) {
            rejectReason = "origin (0,0)";
            lastPoseWasValid = false;
            return;
        }

        // CHECK 2: Reject obviously bad coordinates (outside field)
        if (rawX < -10 || rawX > 155 || rawY < -10 || rawY > 155) {
            rejectReason = "out of bounds";
            lastPoseWasValid = false;
            return;
        }

        // Pose is valid - use it
        lastPoseWasValid = true;
        rejectReason = "";
        lastX = rawX;
        lastY = rawY;

        // Calculate distance to goal
        lastDistance = calculateDistance(rawX, rawY);

        // Calculate RPM and hood from distance
        lastRpm = CalibrationPoints.rpmFromDistance(lastDistance);
        lastHood = CalibrationPoints.hoodFromDistance(lastDistance);
    }

    /**
     * Calculate distance from (x, y) to goal.
     * For red alliance, mirror the coordinates first.
     */
    private double calculateDistance(double x, double y) {
        double effectiveX = x;
        double effectiveY = y;

        // For red alliance, mirror to blue coordinates
        if (isRedAlliance) {
            effectiveX = mirrorAxis - x;
            // Y stays the same in typical mirroring
        }

        return CalibrationPoints.distanceToGoal(effectiveX, effectiveY);
    }

    // Getters
    public double getDistance() { return lastDistance; }
    public double getRpm() { return lastRpm; }
    public double getHood() { return lastHood; }
    public double getX() { return lastX; }
    public double getY() { return lastY; }
    public boolean wasLastPoseValid() { return lastPoseWasValid; }
    public String getRejectReason() { return rejectReason; }

    /**
     * Reset to start position
     */
    public void reset() {
        if (isRedAlliance) {
            Pose redStart = CalibrationPoints.BLUE_START_POSE.mirror(mirrorAxis);
            this.lastX = redStart.getX();
            this.lastY = redStart.getY();
        } else {
            this.lastX = CalibrationPoints.START_X;
            this.lastY = CalibrationPoints.START_Y;
        }
        this.lastDistance = calculateDistance(lastX, lastY);
        this.lastRpm = CalibrationPoints.rpmFromDistance(lastDistance);
        this.lastHood = CalibrationPoints.hoodFromDistance(lastDistance);
    }
}