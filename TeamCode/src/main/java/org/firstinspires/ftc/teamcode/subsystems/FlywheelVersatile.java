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
    private final Pose startPose;
    private final List<DistancePoint> sortedPoints;
    private final double minRpm;
    private final double maxRpm;
    private final double startDistance;

    private boolean isRedAlliance = false;

    private double trimRpm = 0.0;
    private double lastBaseRpm;
    private double lastDistance = 0.0;

    public FlywheelVersatile(FlywheelController flywheel,
                             Pose goalPose,
                             double[][] calibrationData,
                             double minRpm,
                             double maxRpm) {
        this.blueGoalPose = goalPose;
        this.startPose = CalibrationPoints.BLUE_START_POSE;
        this.minRpm = minRpm;
        this.maxRpm = maxRpm;

        // Calculate start distance
        this.startDistance = calculateDistanceBlue(startPose);
        this.lastBaseRpm = minRpm;
        this.lastDistance = startDistance;

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
            return startDistance;
        }
        double dx = pose.getX() - blueGoalPose.getX();
        double dy = pose.getY() - blueGoalPose.getY();
        return Math.hypot(dx, dy);
    }

    private double calculateDistance(Pose robotPose) {
        if (robotPose == null) {
            return startDistance;
        }

        Pose effectivePose = robotPose;

        if (isRedAlliance) {
            effectivePose = robotPose.mirror(MIRROR_AXIS);
        }

        return calculateDistanceBlue(effectivePose);
    }

    /**
     * Check if a pose is valid (not at origin, reasonable values)
     */
    private boolean isValidPose(Pose pose) {
        if (pose == null) {
            return false;
        }
        double x = pose.getX();
        double y = pose.getY();

        // Reject poses at or very near origin
        if (Math.abs(x) < 1.0 && Math.abs(y) < 1.0) {
            return false;
        }

        // Reject poses outside field bounds (0-144 inches typical)
        if (x < -10 || x > 160 || y < -10 || y > 160) {
            return false;
        }

        return true;
    }

    public double getDistanceToGoal(Pose pose) {
        return calculateDistance(pose);
    }

    public double computeBaseRpm(Pose robotPose) {
        // If pose is invalid, return last known good value
        if (!isValidPose(robotPose)) {
            return lastBaseRpm;
        }

        lastDistance = calculateDistance(robotPose);

        if (sortedPoints.isEmpty()) {
            lastBaseRpm = minRpm;
            return lastBaseRpm;
        }

        if (sortedPoints.size() == 1) {
            lastBaseRpm = clamp(sortedPoints.get(0).rpm);
            return lastBaseRpm;
        }

        // Find bracketing points
        DistancePoint lower = null;
        DistancePoint upper = null;

        for (DistancePoint p : sortedPoints) {
            if (p.distance <= lastDistance) {
                lower = p;
            }
            if (p.distance >= lastDistance && upper == null) {
                upper = p;
            }
        }

        // Below all calibration points
        if (lower == null) {
            lastBaseRpm = clamp(sortedPoints.get(0).rpm);
            return lastBaseRpm;
        }

        // Above all calibration points - extrapolate
        if (upper == null) {
            int n = sortedPoints.size();
            DistancePoint p1 = sortedPoints.get(n - 2);
            DistancePoint p2 = sortedPoints.get(n - 1);

            double distRange = p2.distance - p1.distance;
            if (Math.abs(distRange) < 1e-6) {
                lastBaseRpm = clamp(p2.rpm);
            } else {
                double slope = (p2.rpm - p1.rpm) / distRange;
                double extrapolated = p2.rpm + slope * (lastDistance - p2.distance);
                lastBaseRpm = clamp(extrapolated);
            }
            return lastBaseRpm;
        }

        // Interpolate between lower and upper
        double distRange = upper.distance - lower.distance;
        if (Math.abs(distRange) < 1e-6) {
            lastBaseRpm = clamp(lower.rpm);
        } else {
            double t = (lastDistance - lower.distance) / distRange;
            double interpolated = lower.rpm + t * (upper.rpm - lower.rpm);
            lastBaseRpm = clamp(interpolated);
        }

        return lastBaseRpm;
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

    public double getStartDistance() {
        return startDistance;
    }

    private double clamp(double value) {
        return Math.max(minRpm, Math.min(maxRpm, value));
    }
}