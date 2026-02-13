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
    public static final Pose BLUE_GOAL = new Pose(GOAL_X, GOAL_Y, 0);

    // Start position (BLUE)
    public static final double START_X = 20.0;
    public static final double START_Y = 122.0;
    public static final double START_HEADING_DEG = 130.0;
    public static final Pose BLUE_START_POSE = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));

    // Pre-calculated start distance
    public static final double START_DISTANCE = Math.hypot(START_X - GOAL_X, START_Y - GOAL_Y);

    // Distance calibration points
    public static final double DIST_CLOSE = 30.0;
    public static final double DIST_FAR = 140.0;

    // RPM calibration
    public static final double RPM_CLOSE = 2300.0;
    public static final double RPM_FAR = 3750.0;
    public static final double RPM_MIN = 2300.0;
    public static final double RPM_MAX = 4000.0;
    // Aliases for old code
    public static final double FLYWHEEL_MIN_RPM = RPM_MIN;
    public static final double FLYWHEEL_MAX_RPM = RPM_MAX;

    // Hood calibration
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

    // For FlywheelVersatile/HoodVersatile (if still using)
    public static final Pose HOOD_CLOSE_POSE = new Pose(START_X, START_Y, 0);
    public static final Pose HOOD_FAR_POSE = new Pose(72, 12, 0);

    public static final double[][] FLYWHEEL_CALIBRATION_DATA = {
            {20, 122, 130, 2300},
            {60, 125, 135, 2400},
            {48, 96, 135, 2450},
            {72, 120, 167, 2550},
            {60, 82, 135, 2650},
            {95, 120, 135, 2750},
            {72, 72, 135, 2850},
            {52, 14, 135, 3750}
    };

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

    // ==================== HELPER METHODS ====================

    public static double distanceToGoal(double x, double y) {
        double dx = x - GOAL_X;
        double dy = y - GOAL_Y;
        return Math.hypot(dx, dy);
    }

    public static double distanceToGoal(Pose pose) {
        if (pose == null) return START_DISTANCE;
        return distanceToGoal(pose.getX(), pose.getY());
    }

    public static double distanceToGoal(Pose pose, boolean isRedAlliance) {
        if (pose == null) return START_DISTANCE;
        Pose effectivePose = isRedAlliance ? pose.mirror(MIRROR_AXIS) : pose;
        return distanceToGoal(effectivePose.getX(), effectivePose.getY());
    }

    public static double rpmFromDistance(double distance) {
        double slope = (RPM_FAR - RPM_CLOSE) / (DIST_FAR - DIST_CLOSE);
        double rpm = RPM_CLOSE + slope * (distance - DIST_CLOSE);
        return Math.max(RPM_MIN, Math.min(RPM_MAX, rpm));
    }

    public static double hoodFromDistance(double distance) {
        double slope = (HOOD_FAR - HOOD_CLOSE) / (DIST_FAR - DIST_CLOSE);
        double hood = HOOD_CLOSE + slope * (distance - DIST_CLOSE);
        return Math.max(HOOD_MIN, Math.min(HOOD_MAX, hood));
    }

    public static Pose getStartPose(boolean isRed) {
        if (isRed) {
            return BLUE_START_POSE.mirror(MIRROR_AXIS);
        }
        return BLUE_START_POSE;
    }

    public static Pose mirrorPose(Pose pose) {
        return pose.mirror(MIRROR_AXIS);
    }
}