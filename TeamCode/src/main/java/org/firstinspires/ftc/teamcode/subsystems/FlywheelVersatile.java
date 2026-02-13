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

    public FlywheelVersatile(FlywheelController flywheel,
                             Pose goalPose,
                             double[][] calibrationData,
                             double minRpm,
                             double maxRpm) {
        this.blueGoalPose = goalPose;
        this.minRpm = minRpm;
        this.maxRpm = maxRpm;

        // Initialize with start values
        this.lastDistance = CalibrationPoints.START_DISTANCE;
        this.lastBaseRpm = minRpm;

        // Convert raw data to distance points
        this.sortedPoints = new ArrayList<>();
        for (double[] point : calibrationData) {
            if (point.length >= 4) {
                Pose pose = new Pose(point[0], point[1], Math.toRadians(point[2]));
                double dist = calcDistBlue(pose);
                double rpm = point[3];
                sortedPoints.add(new DistancePoint(dist, rpm));
            }
        }

        // Sort by distance ascending
        Collections.sort(sortedPoints, Comparator.comparingDouble(a -> a.distance));
    }

    public void setRedAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
    }

    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**
     * Calculate distance from pose to BLUE goal
     */
    private double calcDistBlue(Pose pose) {
        if (pose == null) return lastDistance;
        double dx = pose.getX() - blueGoalPose.getX();
        double dy = pose.getY() - blueGoalPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Calculate distance with alliance mirroring
     */
    private double calcDist(Pose robotPose) {
        if (robotPose == null) return lastDistance;

        Pose effectivePose = robotPose;
        if (isRedAlliance) {
            effectivePose = robotPose.mirror(MIRROR_AXIS);
        }

        return calcDistBlue(effectivePose);
    }

    public double getDistanceToGoal(Pose pose) {
        return calcDist(pose);
    }

    /**
     * Main computation - SIMPLE and DIRECT
     */
    public double computeBaseRpm(Pose robotPose) {
        // Handle null
        if (robotPose == null) {
            return lastBaseRpm;
        }

        // Get X, Y
        double x = robotPose.getX();
        double y = robotPose.getY();

        // ONLY reject if literally at origin (0,0)
        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
            return lastBaseRpm;
        }

        // Calculate distance
        double dist = calcDist(robotPose);

        // Store for telemetry
        lastDistance = dist;

        // Interpolate RPM from calibration points
        lastBaseRpm = interpolate(dist);

        return lastBaseRpm;
    }

    /**
     * Piecewise linear interpolation
     */
    private double interpolate(double dist) {
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
            if (p.distance <= dist) {
                lower = p;
            }
            if (p.distance >= dist && upper == null) {
                upper = p;
            }
        }

        // Below all points - return first point's RPM
        if (lower == null) {
            return clamp(sortedPoints.get(0).rpm);
        }

        // Above all points - extrapolate from last two
        if (upper == null) {
            int n = sortedPoints.size();
            DistancePoint p1 = sortedPoints.get(n - 2);
            DistancePoint p2 = sortedPoints.get(n - 1);

            double dRange = p2.distance - p1.distance;
            if (dRange < 0.001) {
                return clamp(p2.rpm);
            }

            double slope = (p2.rpm - p1.rpm) / dRange;
            double extrap = p2.rpm + slope * (dist - p2.distance);
            return clamp(extrap);
        }

        // Same point
        if (lower == upper) {
            return clamp(lower.rpm);
        }

        // Interpolate
        double dRange = upper.distance - lower.distance;
        if (dRange < 0.001) {
            return clamp(lower.rpm);
        }

        double t = (dist - lower.distance) / dRange;
        double interp = lower.rpm + t * (upper.rpm - lower.rpm);
        return clamp(interp);
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

    private double clamp(double val) {
        return Math.max(minRpm, Math.min(maxRpm, val));
    }
}