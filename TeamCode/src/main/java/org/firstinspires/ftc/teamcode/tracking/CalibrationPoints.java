package org.firstinspires.ftc.teamcode.tracking;

import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.List;

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

    // ==================== HOOD CALIBRATION POSES (BLUE) ====================
    // Close pose: where hood should be at MINIMUM (0.12)
    // Distance from (20, 122) to goal (0, 144) = sqrt(20^2 + 22^2) = ~30 units
    public static final Pose HOOD_CLOSE_POSE = new Pose(20, 122, 0);

    // Far pose: where hood should be at MAXIMUM (0.48)
    // Distance from (72, 12) to goal (0, 144) = sqrt(72^2 + 132^2) = ~150 units
    public static final Pose HOOD_FAR_POSE = new Pose(72, 12, 0);

    // ==================== HOOD CONSTANTS ====================
    public static final double HOOD_MIN = 0.12;      // Hood position when CLOSE to goal
    public static final double HOOD_MAX = 0.48;      // Hood position when FAR from goal (changed from 0.45!)
    public static final double HOOD_LEFT_STEP = 0.025;
    public static final double HOOD_RIGHT_STEP = 0.01;
    public static final long HOOD_DEBOUNCE_MS = 120L;
    public static final double HOOD_TRIM_STEP = 0.005;

    // Manual preset values (for when auto is disabled)
    public static final double RIGHT_HOOD_CLOSE = 0.16;
    public static final double RIGHT_HOOD_FAR = 0.24;

    // ==================== FLYWHEEL RPM BOUNDS ====================
    public static final double FLYWHEEL_MIN_RPM = 2300;
    public static final double FLYWHEEL_MAX_RPM = 4000;

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

    // ==================== FLYWHEEL CALIBRATION DATA ====================
    /**
     * Raw calibration data: {x, y, heading, rpm}
     * All poses are BLUE coordinates.
     */
    public static final double[][] FLYWHEEL_CALIBRATION_DATA = {
            {48, 96, 135, 2300},
            {60, 125, 135, 2400},
            {60, 82, 135, 2500},
            {72, 72, 135, 2650},
            {72, 120, 167, 2550},
            {95, 120, 135, 2850},
            {52, 14, 135, 3750}
    };

    // ==================== HELPER METHODS ====================

    /**
     * Get the start pose for the specified alliance.
     */
    public static Pose getStartPose(boolean isRedAlliance) {
        if (isRedAlliance) {
            return BLUE_START_POSE.mirror(MIRROR_AXIS);
        }
        return BLUE_START_POSE;
    }

    /**
     * Get the goal pose for the specified alliance.
     */
    public static Pose getGoalPose(boolean isRedAlliance) {
        if (isRedAlliance) {
            return BLUE_GOAL.mirror(MIRROR_AXIS);
        }
        return BLUE_GOAL;
    }

    /**
     * Mirror a pose for red alliance.
     */
    public static Pose mirrorPose(Pose pose) {
        return pose.mirror(MIRROR_AXIS);
    }

    /**
     * Mirror a pose defined by x, y coordinates.
     */
    public static Pose mirrorPose(double x, double y) {
        return new Pose(x, y).mirror(MIRROR_AXIS);
    }

    /**
     * Mirror a pose defined by x, y, heading (degrees).
     */
    public static Pose mirrorPose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg)).mirror(MIRROR_AXIS);
    }
}