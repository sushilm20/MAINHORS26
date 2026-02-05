package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * BallisticsCalculator - Full physics-based trajectory calculation for FTC shooter.
 *
 * Calculates:
 * 1. Optimal launch angle using projectile motion equations
 * 2. Required launch velocity
 * 3. Hood servo position (from angle)
 * 4. Flywheel RPM (from velocity)
 * 5. Turret lead angle for moving targets
 *
 * Based on projectile motion physics:
 * - Launch angle: α = arctan((v² ± sqrt(v⁴ - g(gx² + 2yv²))) / (gx))
 * - For unknown velocity: iterative solution
 * - Range equation: x = (v² * sin(2α)) / g
 */
@Configurable
public class BallisticsCalculator {

    // ==================== PHYSICAL CONSTANTS ====================
    @Sorter(sort = 0) public static double GRAVITY_IN_PER_S2 = 386.09;  // Gravity in inches/s²

    // ==================== ROBOT GEOMETRY ====================
    @Sorter(sort = 1) public static double LAUNCH_HEIGHT_IN = 12.0;     // Height of shooter above ground (inches)
    @Sorter(sort = 2) public static double GOAL_HEIGHT_IN = 33.5;       // Height of goal center (inches) - INTO THE DEEP high basket
    @Sorter(sort = 3) public static double GOAL_DEPTH_IN = 4.0;         // How far into the goal to aim (inches)

    // ==================== HOOD CALIBRATION ====================
    // Two-point calibration: (angle1, servo1) and (angle2, servo2)
    @Sorter(sort = 4) public static double HOOD_ANGLE_MIN_DEG = 25.0;   // Minimum hood angle (degrees)
    @Sorter(sort = 5) public static double HOOD_ANGLE_MAX_DEG = 55.0;   // Maximum hood angle (degrees)
    @Sorter(sort = 6) public static double HOOD_SERVO_AT_MIN = 0.16;    // Servo position at min angle
    @Sorter(sort = 7) public static double HOOD_SERVO_AT_MAX = 0.35;    // Servo position at max angle

    // ==================== FLYWHEEL CALIBRATION ====================
    // Linear fit: RPM = slope * launchSpeed + intercept
    // Derived from empirical measurements
    @Sorter(sort = 8) public static double FLYWHEEL_SPEED_SLOPE = 29.0;      // RPM per inch/s of launch speed
    @Sorter(sort = 9) public static double FLYWHEEL_SPEED_INTERCEPT = 800.0; // Base RPM offset
    @Sorter(sort = 10) public static double FLYWHEEL_RPM_MIN = 2500.0;       // Minimum flywheel RPM
    @Sorter(sort = 11) public static double FLYWHEEL_RPM_MAX = 3500.0;       // Maximum flywheel RPM

    // ==================== DISTANCE CALIBRATION ====================
    // For when using simple distance-based regression as fallback
    @Sorter(sort = 12) public static double DIST_CLOSE_IN = 24.0;
    @Sorter(sort = 13) public static double DIST_FAR_IN = 120.0;
    @Sorter(sort = 14) public static double RPM_AT_CLOSE = 2600.0;
    @Sorter(sort = 15) public static double RPM_AT_FAR = 3000.0;
    @Sorter(sort = 16) public static double HOOD_AT_CLOSE = 0.16;
    @Sorter(sort = 17) public static double HOOD_AT_FAR = 0.24;

    // ==================== TURRET LEAD CALCULATION ====================
    @Sorter(sort = 18) public static double BALL_FLIGHT_TIME_ESTIMATE_S = 0.5;  // Estimated time of flight
    @Sorter(sort = 19) public static boolean ENABLE_LEAD_COMPENSATION = true;

    // ==================== SOLVER SETTINGS ====================
    @Sorter(sort = 20) public static int MAX_ITERATIONS = 20;
    @Sorter(sort = 21) public static double CONVERGENCE_THRESHOLD = 0.01;
    @Sorter(sort = 22) public static boolean USE_PHYSICS_MODEL = true;  // false = use simple linear regression

    // ==================== RESULT CLASS ====================

    /**
     * Container for all calculated ballistic parameters.
     */
    public static class BallisticSolution {
        public boolean valid = false;
        public double distanceIn = 0;           // Horizontal distance to target
        public double heightDiffIn = 0;         // Height difference (goal - launch)
        public double launchAngleDeg = 0;       // Calculated launch angle
        public double launchAngleClamped = 0;   // Clamped to robot limits
        public double launchSpeedInPerS = 0;    // Required launch speed
        public double hoodServoPosition = 0;    // Servo position for hood
        public double flywheelRpm = 0;          // Required flywheel RPM
        public double turretLeadAngleDeg = 0;   // Lead angle for moving target
        public double timeOfFlightS = 0;        // Estimated time of flight
        public String debugInfo = "";           // Debug string

        @Override
        public String toString() {
            return String.format("Dist:%.1f\" Angle:%.1f° Speed:%.0f\"/s Hood:%.3f RPM:%.0f Lead:%.1f°",
                    distanceIn, launchAngleClamped, launchSpeedInPerS,
                    hoodServoPosition, flywheelRpm, turretLeadAngleDeg);
        }
    }

    // ==================== MAIN CALCULATION METHODS ====================

    /**
     * Calculate full ballistic solution for a stationary target.
     *
     * @param horizontalDistanceIn Distance to target in inches
     * @return BallisticSolution with all parameters
     */
    public static BallisticSolution calculate(double horizontalDistanceIn) {
        return calculate(horizontalDistanceIn, 0, 0, 0);
    }

    /**
     * Calculate full ballistic solution with robot velocity compensation.
     *
     * @param horizontalDistanceIn Distance to target in inches
     * @param robotVxInPerS Robot velocity in X (toward target) in inches/s
     * @param robotVyInPerS Robot velocity in Y (perpendicular) in inches/s
     * @param targetBearingDeg Bearing to target from robot heading in degrees
     * @return BallisticSolution with all parameters including turret lead
     */
    public static BallisticSolution calculate(double horizontalDistanceIn,
                                              double robotVxInPerS,
                                              double robotVyInPerS,
                                              double targetBearingDeg) {
        BallisticSolution solution = new BallisticSolution();

        if (horizontalDistanceIn <= 0) {
            solution.debugInfo = "Invalid distance";
            return solution;
        }

        // Use simple regression or full physics
        if (!USE_PHYSICS_MODEL) {
            return calculateSimpleRegression(horizontalDistanceIn);
        }

        // Calculate height difference
        double heightDiff = GOAL_HEIGHT_IN - LAUNCH_HEIGHT_IN;
        solution.distanceIn = horizontalDistanceIn;
        solution.heightDiffIn = heightDiff;

        // Adjust distance for goal depth (aim slightly past the rim)
        double effectiveDistance = horizontalDistanceIn + GOAL_DEPTH_IN;

        // Step 1: Calculate optimal launch angle
        // Using the projectile motion equation for angle given distance and height
        double angleRad = calculateOptimalAngle(effectiveDistance, heightDiff);

        if (Double.isNaN(angleRad)) {
            // Fallback to simple regression if physics fails
            return calculateSimpleRegression(horizontalDistanceIn);
        }

        double angleDeg = Math.toDegrees(angleRad);
        solution.launchAngleDeg = angleDeg;

        // Step 2: Clamp angle to robot's physical limits
        double clampedAngleDeg = Math.max(HOOD_ANGLE_MIN_DEG,
                Math.min(HOOD_ANGLE_MAX_DEG, angleDeg));
        solution.launchAngleClamped = clampedAngleDeg;
        double clampedAngleRad = Math.toRadians(clampedAngleDeg);

        // Step 3: Calculate required launch speed for clamped angle
        double launchSpeed = calculateLaunchSpeed(effectiveDistance, heightDiff, clampedAngleRad);

        if (Double.isNaN(launchSpeed) || launchSpeed <= 0) {
            return calculateSimpleRegression(horizontalDistanceIn);
        }

        solution.launchSpeedInPerS = launchSpeed;

        // Step 4: Calculate time of flight
        double tof = calculateTimeOfFlight(effectiveDistance, launchSpeed, clampedAngleRad);
        solution.timeOfFlightS = tof;

        // Step 5: Convert launch angle to hood servo position
        solution.hoodServoPosition = angleToServoPosition(clampedAngleDeg);

        // Step 6: Convert launch speed to flywheel RPM
        solution.flywheelRpm = launchSpeedToRpm(launchSpeed);

        // Step 7: Calculate turret lead angle for moving targets
        if (ENABLE_LEAD_COMPENSATION && (robotVxInPerS != 0 || robotVyInPerS != 0)) {
            solution.turretLeadAngleDeg = calculateTurretLead(
                    robotVxInPerS, robotVyInPerS, launchSpeed, clampedAngleRad, tof, targetBearingDeg);
        }

        solution.valid = true;
        solution.debugInfo = String.format("Physics model: angle=%.1f->%.1f, speed=%.0f, tof=%.2fs",
                angleDeg, clampedAngleDeg, launchSpeed, tof);

        return solution;
    }

    // ==================== PHYSICS CALCULATIONS ====================

    /**
     * Calculate optimal launch angle for given distance and height difference.
     * Uses the formula: α = arctan((v² - sqrt(v⁴ - g(gx² + 2yv²))) / (gx))
     *
     * Since we don't know v yet, we iterate to find a consistent solution.
     */
    private static double calculateOptimalAngle(double x, double y) {
        // Start with 45 degrees as initial guess (optimal for flat ground)
        double angleRad = Math.toRadians(45);

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            // Calculate speed needed for current angle
            double speed = calculateLaunchSpeed(x, y, angleRad);

            if (Double.isNaN(speed) || speed <= 0) {
                // Try a different angle
                angleRad = Math.toRadians(35 + i * 2);
                continue;
            }

            // Calculate new optimal angle for this speed
            double newAngleRad = calculateAngleForSpeed(x, y, speed);

            if (Double.isNaN(newAngleRad)) {
                continue;
            }

            // Check convergence
            if (Math.abs(newAngleRad - angleRad) < CONVERGENCE_THRESHOLD) {
                return newAngleRad;
            }

            angleRad = newAngleRad;
        }

        // Return best guess if didn't converge
        return angleRad;
    }

    /**
     * Calculate launch angle for a given speed, distance, and height.
     * α = arctan((v² - sqrt(v⁴ - g(gx² + 2yv²))) / (gx))
     * Uses the lower trajectory (minus sign) for flatter shots.
     */
    private static double calculateAngleForSpeed(double x, double y, double v) {
        double g = GRAVITY_IN_PER_S2;
        double v2 = v * v;
        double v4 = v2 * v2;

        double discriminant = v4 - g * (g * x * x + 2 * y * v2);

        if (discriminant < 0) {
            return Double.NaN; // Target unreachable at this speed
        }

        // Use lower trajectory (minus sign) for flatter, faster shots
        double numerator = v2 - Math.sqrt(discriminant);
        double denominator = g * x;

        if (Math.abs(denominator) < 1e-6) {
            return Double.NaN;
        }

        return Math.atan(numerator / denominator);
    }

    /**
     * Calculate required launch speed for given distance, height, and angle.
     * v = sqrt((g * x²) / (2 * cos²(α) * (x * tan(α) - y)))
     */
    private static double calculateLaunchSpeed(double x, double y, double angleRad) {
        double g = GRAVITY_IN_PER_S2;
        double cosA = Math.cos(angleRad);
        double tanA = Math.tan(angleRad);

        double denominator = 2 * cosA * cosA * (x * tanA - y);

        if (denominator <= 0) {
            return Double.NaN; // Invalid trajectory
        }

        double v2 = (g * x * x) / denominator;

        if (v2 <= 0) {
            return Double.NaN;
        }

        return Math.sqrt(v2);
    }

    /**
     * Calculate time of flight.
     * t = x / (v * cos(α))
     */
    private static double calculateTimeOfFlight(double x, double v, double angleRad) {
        double vx = v * Math.cos(angleRad);
        if (Math.abs(vx) < 1e-6) {
            return BALL_FLIGHT_TIME_ESTIMATE_S; // Fallback
        }
        return x / vx;
    }

    /**
     * Calculate turret lead angle for moving robot.
     *
     * The idea: while the ball is in flight, the robot moves.
     * We need to aim ahead of where we currently are relative to the target.
     */
    private static double calculateTurretLead(double robotVx, double robotVy,
                                              double launchSpeed, double launchAngleRad,
                                              double timeOfFlight, double bearingDeg) {
        // Horizontal component of launch velocity
        double vLaunchHorizontal = launchSpeed * Math.cos(launchAngleRad);

        if (Math.abs(vLaunchHorizontal) < 1e-6) {
            return 0;
        }

        // Robot's perpendicular velocity component (causes lateral drift)
        // Convert bearing to radians for proper decomposition
        double bearingRad = Math.toRadians(bearingDeg);

        // Perpendicular velocity component (causes the need for lead)
        double vPerp = -robotVx * Math.sin(bearingRad) + robotVy * Math.cos(bearingRad);

        // Lead angle: arctan(vPerp * tof / (vLaunch * tof)) = arctan(vPerp / vLaunch)
        // But we also account for the distance traveled during flight
        double leadRad = Math.atan2(vPerp * timeOfFlight, vLaunchHorizontal * timeOfFlight);

        return Math.toDegrees(leadRad);
    }

    // ==================== CONVERSION METHODS ====================

    /**
     * Convert launch angle (degrees) to hood servo position.
     * Linear interpolation between calibration points.
     */
    public static double angleToServoPosition(double angleDeg) {
        // Clamp angle to valid range
        angleDeg = Math.max(HOOD_ANGLE_MIN_DEG, Math.min(HOOD_ANGLE_MAX_DEG, angleDeg));

        // Linear interpolation
        double angleRange = HOOD_ANGLE_MAX_DEG - HOOD_ANGLE_MIN_DEG;
        double servoRange = HOOD_SERVO_AT_MAX - HOOD_SERVO_AT_MIN;

        if (Math.abs(angleRange) < 1e-6) {
            return HOOD_SERVO_AT_MIN;
        }

        double t = (angleDeg - HOOD_ANGLE_MIN_DEG) / angleRange;
        return HOOD_SERVO_AT_MIN + t * servoRange;
    }

    /**
     * Convert hood servo position back to angle (for telemetry).
     */
    public static double servoPositionToAngle(double servoPos) {
        double servoRange = HOOD_SERVO_AT_MAX - HOOD_SERVO_AT_MIN;
        double angleRange = HOOD_ANGLE_MAX_DEG - HOOD_ANGLE_MIN_DEG;

        if (Math.abs(servoRange) < 1e-6) {
            return HOOD_ANGLE_MIN_DEG;
        }

        double t = (servoPos - HOOD_SERVO_AT_MIN) / servoRange;
        return HOOD_ANGLE_MIN_DEG + t * angleRange;
    }

    /**
     * Convert launch speed (in/s) to flywheel RPM.
     * Uses empirically-derived linear relationship.
     */
    public static double launchSpeedToRpm(double speedInPerS) {
        double rpm = FLYWHEEL_SPEED_SLOPE * speedInPerS + FLYWHEEL_SPEED_INTERCEPT;
        return Math.max(FLYWHEEL_RPM_MIN, Math.min(FLYWHEEL_RPM_MAX, rpm));
    }

    /**
     * Convert flywheel RPM back to launch speed (for telemetry/debug).
     */
    public static double rpmToLaunchSpeed(double rpm) {
        if (Math.abs(FLYWHEEL_SPEED_SLOPE) < 1e-6) {
            return 0;
        }
        return (rpm - FLYWHEEL_SPEED_INTERCEPT) / FLYWHEEL_SPEED_SLOPE;
    }

    // ==================== SIMPLE REGRESSION FALLBACK ====================

    /**
     * Simple linear regression fallback when physics model fails.
     * Uses the two-point calibration data.
     */
    public static BallisticSolution calculateSimpleRegression(double distanceIn) {
        BallisticSolution solution = new BallisticSolution();
        solution.distanceIn = distanceIn;
        solution.heightDiffIn = GOAL_HEIGHT_IN - LAUNCH_HEIGHT_IN;

        // Clamp distance to calibration range
        double clampedDist = Math.max(DIST_CLOSE_IN, Math.min(DIST_FAR_IN, distanceIn));

        // Linear interpolation factor
        double t = (clampedDist - DIST_CLOSE_IN) / (DIST_FAR_IN - DIST_CLOSE_IN);
        t = Math.max(0, Math.min(1, t));

        // Interpolate RPM
        solution.flywheelRpm = RPM_AT_CLOSE + t * (RPM_AT_FAR - RPM_AT_CLOSE);
        solution.flywheelRpm = Math.max(FLYWHEEL_RPM_MIN, Math.min(FLYWHEEL_RPM_MAX, solution.flywheelRpm));

        // Interpolate hood position
        solution.hoodServoPosition = HOOD_AT_CLOSE + t * (HOOD_AT_FAR - HOOD_AT_CLOSE);

        // Estimate angle from servo position
        solution.launchAngleClamped = servoPositionToAngle(solution.hoodServoPosition);
        solution.launchAngleDeg = solution.launchAngleClamped;

        // Estimate launch speed from RPM
        solution.launchSpeedInPerS = rpmToLaunchSpeed(solution.flywheelRpm);

        // Estimate time of flight
        solution.timeOfFlightS = BALL_FLIGHT_TIME_ESTIMATE_S;

        solution.valid = true;
        solution.debugInfo = "Simple regression fallback";

        return solution;
    }

    // ==================== UTILITY METHODS ====================

    /**
     * Check if a target is reachable with current robot configuration.
     */
    public static boolean isTargetReachable(double distanceIn) {
        BallisticSolution solution = calculate(distanceIn);
        return solution.valid;
    }

    /**
     * Get maximum theoretical range at optimal angle (45°).
     */
    public static double getMaxRange(double launchSpeedInPerS) {
        // R = v² / g (at 45° on flat ground)
        // With height difference, it's more complex, but this gives an estimate
        return (launchSpeedInPerS * launchSpeedInPerS) / GRAVITY_IN_PER_S2;
    }

    /**
     * Get regression info string for telemetry.
     */
    public static String getCalibrationInfo() {
        return String.format("Hood: %.1f°->%.3f, %.1f°->%.3f | RPM: %.0f\"/s->%.0f, slope=%.1f",
                HOOD_ANGLE_MIN_DEG, HOOD_SERVO_AT_MIN,
                HOOD_ANGLE_MAX_DEG, HOOD_SERVO_AT_MAX,
                rpmToLaunchSpeed(FLYWHEEL_RPM_MIN), FLYWHEEL_RPM_MIN,
                FLYWHEEL_SPEED_SLOPE);
    }
}