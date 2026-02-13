package org.firstinspires.ftc.teamcode.tracking;

import com.pedropathing.geometry.Pose;

/**
 * All calibration data in one place.
 * All poses in BLUE coordinates.
 */
public class CalibrationPoints {

    // Field
    public static final double MIRROR_AXIS = 146.0;

    // Goal position (BLUE)
    public static final double GOAL_X = 0.0;
    public static final double GOAL_Y = 144.0;

    // Start position (BLUE)
    public static final double START_X = 20.0;
    public static final double START_Y = 122.0;
    public static final double START_HEADING_DEG = 130.0;

    public static final Pose BLUE_START_POSE = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    public static final Pose BLUE_GOAL = new Pose(GOAL_X, GOAL_Y, 0);

    // RPM calibration: at 30 inches = 2300 RPM, at 140 inches = 3750 RPM
    public static final double DIST_CLOSE = 30.0;
    public static final double DIST_FAR = 140.0;
    public static final double RPM_CLOSE = 2300.0;
    public static final double RPM_FAR = 3750.0;
    public static final double RPM_MIN = 2300.0;
    public static final double RPM_MAX = 4000.0;

    // Hood calibration: at 30 inches = 0.12, at 140 inches = 0.48
    public static final double HOOD_CLOSE = 0.12;
    public static final double HOOD_FAR = 0.48;
    public static final double HOOD_MIN = 0.12;
    public static final double HOOD_MAX = 0.48;

    // Manual hood presets
    public static final double RIGHT_HOOD_CLOSE = 0.16;
    public static final double RIGHT_HOOD_FAR = 0.24;

    // Hood control
    public static final double HOOD_LEFT_STEP = 0.025;
    public static final double HOOD_RIGHT_STEP = 0.01;
    public static final long HOOD_DEBOUNCE_MS = 120L;
    public static final double HOOD_TRIM_STEP = 0.005;

    // Gate/Intake
    public static final double GATE_OPEN = 0.67;
    public static final double GATE_CLOSED = 0.467;
    public static final long INTAKE_DURATION_MS = 1050;
    public static final long CLAW_TRIGGER_BEFORE_END_MS = 400;
    public static final double INTAKE_SEQUENCE_POWER = 1.0;

    // Claw
    public static final double CLAW_OPEN = 0.63;
    public static final double CLAW_CLOSED = 0.2;
    public static final long CLAW_CLOSE_MS = 500L;

    /**
     * Calculate distance from a pose to the BLUE goal
     */
    public static double distanceToGoal(double x, double y) {
        double dx = x - GOAL_X;
        double dy = y - GOAL_Y;
        return Math.hypot(dx, dy);
    }

    /**
     * Calculate RPM from distance using linear interpolation
     */
    public static double rpmFromDistance(double distance) {
        // Linear interpolation: rpm = slope * distance + intercept
        double slope = (RPM_FAR - RPM_CLOSE) / (DIST_FAR - DIST_CLOSE);
        double rpm = RPM_CLOSE + slope * (distance - DIST_CLOSE);
        return Math.max(RPM_MIN, Math.min(RPM_MAX, rpm));
    }

    /**
     * Calculate hood position from distance using linear interpolation
     */
    public static double hoodFromDistance(double distance) {
        double slope = (HOOD_FAR - HOOD_CLOSE) / (DIST_FAR - DIST_CLOSE);
        double hood = HOOD_CLOSE + slope * (distance - DIST_CLOSE);
        return Math.max(HOOD_MIN, Math.min(HOOD_MAX, hood));
    }

    /**
     * Get mirrored start pose for red alliance
     */
    public static Pose getStartPose(boolean isRed) {
        if (isRed) {
            return BLUE_START_POSE.mirror(MIRROR_AXIS);
        }
        return BLUE_START_POSE;
    }
}