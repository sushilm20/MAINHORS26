package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import java.util.List;

/**
 * Computes target RPM from robot pose relative to a goal, using linear regression
 * over calibration points (distance -> RPM) plus driver trim.
 *
 * Adds a flat radius around the goal: inside this radius RPM is held at minRpm.
 * Outside, RPM increases with distance based on calibration.
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

    private final Flywheel flywheel;
    private final Pose goalPose;
    private final List<CalibrationPoint> calibration;
    private final double minRpm;
    private final double maxRpm;

    // Flat radius (in field units). If distance <= flatRadius, RPM = minRpm.
    private double flatRadius = 0.0;

    private double slope = 0.0;
    private double intercept = 0.0;
    private double trimRpm = 0.0;
    private double lastBaseRpm = 0.0;
    private double lastDistance = 0.0;

    // Existing constructor (keeps backward compatibility, flatRadius=0)
    public FlywheelVersatile(Flywheel flywheel,
                             Pose goalPose,
                             List<CalibrationPoint> calibration,
                             double minRpm,
                             double maxRpm) {
        this(flywheel, goalPose, calibration, minRpm, maxRpm, 0.0);
    }

    // New constructor with configurable flat radius
    public FlywheelVersatile(Flywheel flywheel,
                             Pose goalPose,
                             List<CalibrationPoint> calibration,
                             double minRpm,
                             double maxRpm,
                             double flatRadius) {
        this.flywheel = flywheel;
        this.goalPose = goalPose;
        this.calibration = calibration;
        this.minRpm = minRpm;
        this.maxRpm = maxRpm;
        this.flatRadius = Math.max(0.0, flatRadius);
        this.lastBaseRpm = clamp((minRpm + maxRpm) / 2.0);
        computeRegression();
    }

    public void setFlatRadius(double flatRadius) {
        this.flatRadius = Math.max(0.0, flatRadius);
        computeRegression();
    }

    private void computeRegression() {
        int n = calibration.size();
        if (n < 2) {
            slope = 0.0;
            intercept = clamp(minRpm);
            return;
        }

        double[] adj = new double[n]; // adjusted distances (distance - flatRadius, floored at 0)
        double meanX = 0.0;
        double meanY = 0.0;
        for (int i = 0; i < n; i++) {
            double dist = distanceToGoal(calibration.get(i).pose);
            double d = Math.max(0.0, dist - flatRadius);
            adj[i] = d;
            meanX += d;
            meanY += calibration.get(i).rpm;
        }
        meanX /= n;
        meanY /= n;

        double cov = 0.0;
        double var = 0.0;
        for (int i = 0; i < n; i++) {
            double dx = adj[i] - meanX;
            cov += dx * (calibration.get(i).rpm - meanY);
            var += dx * dx;
        }

        if (Math.abs(var) < 1e-6) {
            slope = 0.0;
            intercept = clamp(meanY);
        } else {
            slope = cov / var;
            intercept = meanY - slope * meanX;
        }
    }

    private double clamp(double value) {
        return Math.max(minRpm, Math.min(maxRpm, value));
    }

    private double distanceToGoal(Pose pose) {
        double dx = pose.getX() - goalPose.getX();
        double dy = pose.getY() - goalPose.getY();
        return Math.hypot(dx, dy);
    }

    public double getDistanceToGoal(Pose pose) {
        return distanceToGoal(pose);
    }

    public double computeBaseRpm(Pose robotPose) {
        if (robotPose == null) {
            return lastBaseRpm; // fallback to last known/base
        }
        lastDistance = distanceToGoal(robotPose);

        // Flat radius behavior: hold minRpm when inside radius
        if (lastDistance <= flatRadius) {
            lastBaseRpm = minRpm;
            return lastBaseRpm;
        }

        double adjDist = Math.max(0.0, lastDistance - flatRadius);
        double raw = slope * adjDist + intercept;
        lastBaseRpm = clamp(raw);
        return lastBaseRpm;
    }

    public double getFinalTargetRPM(Pose robotPose) {
        double base = computeBaseRpm(robotPose);
        return clamp(base + trimRpm);
    }

    public void adjustTrim(double delta) {
        trimRpm += delta;
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
}