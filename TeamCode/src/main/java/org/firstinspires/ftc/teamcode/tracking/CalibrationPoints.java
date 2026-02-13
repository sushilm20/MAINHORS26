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

    // Pre-calculated start distance: sqrt(20² + 22²) ≈ 29.7
    public static final double START_DISTANCE = 29.7;

    // ==================== HOOD CALIBRATION ====================
    // Close = start pose, Far = max range pose
    public static final Pose HOOD_CLOSE_POSE = new Pose(20, 122, 0);   // ~30 units from goal
    public static final Pose HOOD_FAR_POSE = new Pose(72, 12, 0);      // ~150 units from goal

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

    /**
     * Calibration data: {x, y, heading, rpm}
     * MUST be sorted by increasing distance to goal for proper interpolation!
     *
     * Distance calculations (goal at 0, 144):
     * (20, 122)  → sqrt(20² + 22²)  = 29.7  → 2300 RPM
     * (60, 125)  → sqrt(60² + 19²)  = 62.9  → 2400 RPM
     * (48, 96)   → sqrt(48² + 48²)  = 67.9  → 2450 RPM (adjusted for monotonic)
     * (72, 120)  → sqrt(72² + 24²)  = 75.9  → 2550 RPM
     * (60, 82)   → sqrt(60² + 62²)  = 86.3  → 2650 RPM (adjusted for monotonic)
     * (95, 120)  → sqrt(95² + 24²)  = 98.0  → 2750 RPM (adjusted for monotonic)
     * (72, 72)   → sqrt(72² + 72²)  = 101.8 → 2850 RPM (adjusted for monotonic)
     * (52, 14)   → sqrt(52² + 130²) = 140.0 → 3750 RPM
     */
    public static final double[][] FLYWHEEL_CALIBRATION_DATA = {
            // Sorted by distance: closest to farthest
            // {x, y, heading, rpm}
            {20, 122, 130, 2300},   // dist ~30
            {60, 125, 135, 2400},   // dist ~63
            {48, 96, 135, 2450},    // dist ~68 (was 2350, adjusted to be monotonic)
            {72, 120, 167, 2550},   // dist ~76
            {60, 82, 135, 2650},    // dist ~86 (was 2500, adjusted to be monotonic)
            {95, 120, 135, 2750},   // dist ~98 (was 2850, adjusted to be monotonic)
            {72, 72, 135, 2850},    // dist ~102 (was 2650, adjusted to be monotonic)
            {52, 14, 135, 3750}     // dist ~140
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
        if (pose == null) return START_DISTANCE;
        double dx = pose.getX() - BLUE_GOAL.getX();
        double dy = pose.getY() - BLUE_GOAL.getY();
        return Math.hypot(dx, dy);
    }

    public static double distanceToGoal(Pose pose, boolean isRedAlliance) {
        if (pose == null) return START_DISTANCE;
        Pose effectivePose = isRedAlliance ? pose.mirror(MIRROR_AXIS) : pose;
        double dx = effectivePose.getX() - BLUE_GOAL.getX();
        double dy = effectivePose.getY() - BLUE_GOAL.getY();
        return Math.hypot(dx, dy);
    }

    public static Pose mirrorPose(Pose pose) {
        return pose.mirror(MIRROR_AXIS);
    }
}