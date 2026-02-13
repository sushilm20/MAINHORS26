package org.firstinspires.ftc.teamcode.tracking;

import com.pedropathing.geometry.Pose;

/**
 * Central storage for all calibration data.
 * All poses are in BLUE coordinates - auto mirrors for red alliance.
 */
public class CalibrationPoints {

    // ==================== FIELD CONSTANTS ====================
    public static final double MIRROR_AXIS = 146.0;

    // ==================== GOAL POSES (BLUE) ====================
    public static final Pose BLUE_GOAL = new Pose(0, 144, 0);

    // ==================== START POSES (BLUE) ====================
    public static final Pose BLUE_START_POSE = new Pose(20, 122, Math.toRadians(130));

    // Pre-calculated start distance (for initialization)
    public static final double START_DISTANCE = Math.hypot(
            BLUE_START_POSE.getX() - BLUE_GOAL.getX(),
            BLUE_START_POSE.getY() - BLUE_GOAL.getY()
    );  // Should be ~30 units

    // ==================== HOOD CALIBRATION ====================
    public static final Pose HOOD_CLOSE_POSE = new Pose(20, 122, 0);
    public static final Pose HOOD_FAR_POSE = new Pose(72, 12, 0);

    // Hood servo positions
    public static final double HOOD_MIN = 0.12;
    public static final double HOOD_MAX = 0.48;

    // Hood control constants
    public static final double HOOD_LEFT_STEP = 0.025;
    public static final double HOOD_RIGHT_STEP = 0.01;
    public static final long HOOD_DEBOUNCE_MS = 120L;
    public static final double HOOD_TRIM_STEP = 0.005;

    // Manual preset values
    public static final double RIGHT_HOOD_CLOSE = 0.16;
    public static final double RIGHT_HOOD_FAR = 0.24;

    // ==================== FLYWHEEL CALIBRATION ====================
    public static final double FLYWHEEL_MIN_RPM = 2300;
    public static final double FLYWHEEL_MAX_RPM = 4000;

    // Calibration data: {x, y, heading, rpm}
    public static final double[][] FLYWHEEL_CALIBRATION_DATA = {
            {20, 122, 130, 2300},   // Start pose - minimum RPM
            {48, 96, 135, 2350},
            {60, 125, 135, 2400},
            {60, 82, 135, 2500},
            {72, 120, 167, 2550},
            {72, 72, 135, 2650},
            {95, 120, 135, 2850},
            {52, 14, 135, 3750}
    };

    // ==================== GATE/INTAKE CONSTANTS ====================
    public static final double GATE_OPEN = 0.67;
    public static final double GATE_CLOSED = 0.467;
    public static final long INTAKE_DURATION_MS = 1050;
    public static final long CLAW_TRIGGER_BEFORE_END_MS = 400;
    public static final double INTAKE_SEQUENCE_POWER = 1.0;

    // ==================== CLAW CONSTANTS ====================
    public static final double CLAW_OPEN = 0.63;
    public static final double CLAW_CLOSED = 0.2;
    public static final long CLAW_CLOSE_MS = 500L;

    // ==================== POSE VALIDATION CONSTANTS ====================
    // Maximum distance jump per cycle - VERY generous to allow first reading
    public static final double MAX_DISTANCE_JUMP = 50.0;
    public static final double MAX_POSE_MOVEMENT_PER_CYCLE = 30.0;

    // ==================== HELPER METHODS ====================

    public static Pose getStartPose(boolean isRedAlliance) {
        if (isRedAlliance) {
            return BLUE_START_POSE.mirror(MIRROR_AXIS);
        }
        return BLUE_START_POSE;
    }

    public static Pose getGoalPose(boolean isRedAlliance) {
        if (isRedAlliance) {
            return BLUE_GOAL.mirror(MIRROR_AXIS);
        }
        return BLUE_GOAL;
    }

    public static double distanceToGoal(Pose pose) {
        double dx = pose.getX() - BLUE_GOAL.getX();
        double dy = pose.getY() - BLUE_GOAL.getY();
        return Math.hypot(dx, dy);
    }

    public static double distanceToGoal(Pose pose, boolean isRedAlliance) {
        Pose effectivePose = isRedAlliance ? pose.mirror(MIRROR_AXIS) : pose;
        double dx = effectivePose.getX() - BLUE_GOAL.getX();
        double dy = effectivePose.getY() - BLUE_GOAL.getY();
        return Math.hypot(dx, dy);
    }

    public static Pose mirrorPose(Pose pose) {
        return pose.mirror(MIRROR_AXIS);
    }
}