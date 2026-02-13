package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.tracking.CalibrationPoints;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Computes target RPM from robot pose using piecewise linear interpolation.
 * CLOSE to goal = LOW RPM, FAR from goal = HIGH RPM
 */
public class FlywheelVersatile {

    private static class DistancePoint {
        final double distance;
        final double rpm;

        DistancePoint(double distance, double rpm) {
            this.distance = distance;
            this.rpm = rpm;
        }
    }

    private static final double MIRROR_AXIS = CalibrationPoints.MIRROR_AXIS;

    private final Pose blueGoalPose;
    private final List<DistancePoint> sortedPoints;
    private final double minRpm;
    private final double maxRpm;

    private boolean isRedAlliance = false;

    private double trimRpm = 0.0;
    private double lastBaseRpm;
    private double lastDistance;
    private double lastValidDistance;

    // Track number of updates for debugging
    private int updateCount = 0;
    private String lastRejectReason = "";

    public FlywheelVersatile(FlywheelController flywheel,
                             Pose goalPose,
                             double[][] calibrationData,
                             double minRpm,
                             double maxRpm) {
        this.blueGoalPose = goalPose;
        this.minRpm = minRpm;
        this.maxRpm = maxRpm;

        // Initialize to START_POSE distance
        this.lastDistance = CalibrationPoints.START_DISTANCE;
        this.lastValidDistance = CalibrationPoints.START_DISTANCE;
        this.lastBaseRpm = minRpm;

        // Convert raw data to distance points
        this.sortedPoints = new ArrayList<>();
        for (double[] point : calibrationData) {
            if (point.length >= 4) {
                Pose pose = new Pose(point[0], point[1], Math.toRadians(point[2]));
                double dist = calculateDistanceBlue(pose);
                double rpm = point[3];
                sortedPoints.add(new DistancePoint(dist, rpm));
            }
        }

        Collections.sort(sortedPoints, Comparator.comparingDouble(a -> a.distance));
    }

    public void setRedAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
    }

    public boolean isRedAlliance() {
        return isRedAlliance;
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
     * Compute base RPM from pose - SIMPLIFIED VERSION
     * Removes overly aggressive validation that was blocking updates
     */
    public double computeBaseRpm(Pose robotPose) {
        updateCount++;
        lastRejectReason = "";

        // Null check
        if (robotPose == null) {
            lastRejectReason = "null pose";
            return lastBaseRpm;
        }

        double x = robotPose.getX();
        double y = robotPose.getY();

        // Only reject obvious bad poses (at origin)
        if (Math.abs(x) < 0.5 && Math.abs(y) < 0.5) {
            lastRejectReason = "origin pose";
            return lastBaseRpm;
        }

        // Calculate distance
        double newDistance = calculateDistance(robotPose);

        // Basic sanity check - distance should be positive and reasonable
        if (newDistance < 5.0 || newDistance > 250.0) {
            lastRejectReason = "distance out of range: " + newDistance;
            return lastBaseRpm;
        }

        // ACCEPT the distance - update tracking
        lastDistance = newDistance;
        lastValidDistance = newDistance;

        // Calculate RPM from distance using interpolation
        lastBaseRpm = interpolateRpm(newDistance);

        return lastBaseRpm;
    }

    /**
     * Interpolate RPM from distance using calibration points
     */
    private double interpolateRpm(double distance) {
        if (sortedPoints.isEmpty()) {
            return minRpm;
        }

        if (sortedPoints.size() == 1) {
            return clamp(sortedPoints.get(0).rpm);
        }

        // Find bracketing points
        DistancePoint lower = null;
        DistancePoint upper = null;

        for (DistancePoint p : sortedPoints) {
            if (p.distance <= distance) {
                lower = p;
            }
            if (p.distance >= distance && upper == null) {
                upper = p;
            }
        }

        // Below all calibration points
        if (lower == null) {
            return clamp(sortedPoints.get(0).rpm);
        }

        // Above all calibration points - extrapolate
        if (upper == null) {
            int n = sortedPoints.size();
            DistancePoint p1 = sortedPoints.get(n - 2);
            DistancePoint p2 = sortedPoints.get(n - 1);

            double distRange = p2.distance - p1.distance;
            if (Math.abs(distRange) < 1e-6) {
                return clamp(p2.rpm);
            }

            double slope = (p2.rpm - p1.rpm) / distRange;
            double extrapolated = p2.rpm + slope * (distance - p2.distance);
            return clamp(extrapolated);
        }

        // Interpolate between lower and upper
        double distRange = upper.distance - lower.distance;
        if (Math.abs(distRange) < 1e-6) {
            return clamp(lower.rpm);
        }

        double t = (distance - lower.distance) / distRange;
        double interpolated = lower.rpm + t * (upper.rpm - lower.rpm);
        return clamp(interpolated);
    }

    public double getFinalTargetRPM(Pose robotPose) {
        double base = computeBaseRpm(robotPose);
        return clamp(base + trimRpm);
    }

    public void adjustTrim(double delta) {
        trimRpm += delta;
    }

    public void resetTrim() {
        trimRpm = 0.0;
    }

    public double getTrimRpm() {
        return trimRpm;
    }

    public double getLastBaseRpm() {
        return lastBaseRpm;
    }

    public double getLastDistance() {
        return lastDistance;
    }

    public double getLastValidDistance() {
        return lastValidDistance;
    }

    public int getUpdateCount() {
        return updateCount;
    }

    public String getLastRejectReason() {
        return lastRejectReason;
    }

    private double clamp(double value) {
        return Math.max(minRpm, Math.min(maxRpm, value));
    }
}