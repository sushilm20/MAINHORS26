package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Computes target RPM from robot pose relative to a goal using piecewise linear interpolation.
 * Supports both BLUE and RED alliance - just call setRedAlliance(true) and it auto-mirrors!
 */
public class FlywheelVersatile {

    public static class CalibrationPoint {
        public final Pose pose;
        public final double rpm;
        public CalibrationPoint(Pose pose, double rpm) {
            this.pose = pose;
            this.rpm = rpm;
        }
    }

    private static class DistancePoint {
        final double distance;
        final double rpm;
        DistancePoint(double distance, double rpm) {
            this.distance = distance;
            this.rpm = rpm;
        }
    }

    private static final double MIRROR_AXIS = 146.0;  // Your mirror axis

    private final FlywheelController flywheel;
    private final Pose blueGoalPose;
    private final List<DistancePoint> sortedPoints;
    private final double minRpm;
    private final double maxRpm;
    private double flatRadius = 0.0;

    private boolean isRedAlliance = false;
    private Pose effectiveGoalPose;  // Cached goal pose for current alliance

    private double trimRpm = 0.0;
    private double lastBaseRpm = 0.0;
    private double lastDistance = 0.0;

    public FlywheelVersatile(FlywheelController flywheel,
                             Pose goalPose,
                             List<CalibrationPoint> calibration,
                             double minRpm,
                             double maxRpm) {
        this(flywheel, goalPose, calibration, minRpm, maxRpm, 0.0);
    }

    public FlywheelVersatile(FlywheelController flywheel,
                             Pose goalPose,
                             List<CalibrationPoint> calibration,
                             double minRpm,
                             double maxRpm,
                             double flatRadius) {
        this.flywheel = flywheel;
        this.blueGoalPose = goalPose;
        this.effectiveGoalPose = goalPose;  // Default to blue
        this.minRpm = minRpm;
        this.maxRpm = maxRpm;
        this.flatRadius = Math.max(0.0, flatRadius);
        this.lastBaseRpm = clamp((minRpm + maxRpm) / 2.0);

        // Convert poses to distances (using BLUE coordinates) and sort
        this.sortedPoints = new ArrayList<>();
        for (CalibrationPoint p : calibration) {
            double dist = distanceToGoalBlue(p.pose);
            sortedPoints.add(new DistancePoint(dist, p.rpm));
        }
        Collections.sort(sortedPoints, Comparator.comparingDouble(a -> a.distance));
    }

    /**
     * Set whether this is red alliance.
     * When true, robot poses are auto-mirrored before distance calculation.
     */
    public void setRedAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
        // Cache the effective goal pose
        if (isRed) {
            this.effectiveGoalPose = blueGoalPose.mirror(MIRROR_AXIS);
        } else {
            this.effectiveGoalPose = blueGoalPose;
        }
    }

    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    public void setFlatRadius(double flatRadius) {
        this.flatRadius = Math.max(0.0, flatRadius);
    }

    public Pose getEffectiveGoalPose() {
        return effectiveGoalPose;
    }

    /**
     * Distance calculation using BLUE coordinates (for calibration)
     */
    private double distanceToGoalBlue(Pose pose) {
        double dx = pose.getX() - blueGoalPose.getX();
        double dy = pose.getY() - blueGoalPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Distance calculation that auto-mirrors for red alliance
     */
    private double distanceToGoal(Pose robotPose) {
        Pose effectivePose = robotPose;

        // Mirror the robot pose if red alliance
        if (isRedAlliance) {
            effectivePose = robotPose.mirror(MIRROR_AXIS);
        }

        // Always calculate distance to BLUE goal (calibration is in blue coords)
        double dx = effectivePose.getX() - blueGoalPose.getX();
        double dy = effectivePose.getY() - blueGoalPose.getY();
        return Math.hypot(dx, dy);
    }

    public double getDistanceToGoal(Pose pose) {
        return distanceToGoal(pose);
    }

    public double computeBaseRpm(Pose robotPose) {
        if (robotPose == null) {
            return lastBaseRpm;
        }

        lastDistance = distanceToGoal(robotPose);

        if (lastDistance <= flatRadius) {
            lastBaseRpm = minRpm;
            return lastBaseRpm;
        }

        double adjDist = lastDistance - flatRadius;

        if (sortedPoints.isEmpty()) {
            lastBaseRpm = minRpm;
            return lastBaseRpm;
        }

        if (sortedPoints.size() == 1) {
            lastBaseRpm = clamp(sortedPoints.get(0).rpm);
            return lastBaseRpm;
        }

        // Find bracketing points for interpolation
        DistancePoint lower = null;
        DistancePoint upper = null;

        for (DistancePoint p : sortedPoints) {
            double pDist = Math.max(0.0, p.distance - flatRadius);

            if (pDist <= adjDist) {
                lower = new DistancePoint(pDist, p.rpm);
            }
            if (pDist >= adjDist && upper == null) {
                upper = new DistancePoint(pDist, p.rpm);
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
            double d1 = Math.max(0.0, p1.distance - flatRadius);
            double d2 = Math.max(0.0, p2.distance - flatRadius);

            if (Math.abs(d2 - d1) < 1e-6) {
                lastBaseRpm = clamp(p2.rpm);
            } else {
                double slope = (p2.rpm - p1.rpm) / (d2 - d1);
                double extrapolated = p2.rpm + slope * (adjDist - d2);
                lastBaseRpm = clamp(extrapolated);
            }
            return lastBaseRpm;
        }

        // Interpolate between lower and upper
        if (Math.abs(upper.distance - lower.distance) < 1e-6) {
            lastBaseRpm = clamp(lower.rpm);
        } else {
            double t = (adjDist - lower.distance) / (upper.distance - lower.distance);
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

    private double clamp(double value) {
        return Math.max(minRpm, Math.min(maxRpm, value));
    }
}