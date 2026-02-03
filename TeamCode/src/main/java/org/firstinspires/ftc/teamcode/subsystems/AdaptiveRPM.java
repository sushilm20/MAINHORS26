package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * AdaptiveRPM - Adaptive flywheel and hood control using ballistics or simple regression.
 *
 * Supports two modes:
 * 1. Physics-based: Uses BallisticsCalculator for full trajectory computation
 * 2. Simple regression: Linear interpolation between calibration points
 *
 * Also handles smoothing, hysteresis, and fallback logic.
 */
@Configurable
public class AdaptiveRPM {

    // ==================== MODE SELECTION ====================
    @Sorter(sort = 0) public static boolean USE_PHYSICS_MODEL = false;  // true = physics, false = regression

    // ==================== SIMPLE REGRESSION CALIBRATION ====================
    @Sorter(sort = 1) public static double DIST_CLOSE_INCHES = 24.0;
    @Sorter(sort = 2) public static double RPM_CLOSE = 2600.0;
    @Sorter(sort = 3) public static double DIST_FAR_INCHES = 120.0;
    @Sorter(sort = 4) public static double RPM_FAR = 3000.0;

    @Sorter(sort = 5) public static double RPM_MIN = 2400.0;
    @Sorter(sort = 6) public static double RPM_MAX = 3500.0;
    @Sorter(sort = 7) public static double RPM_FALLBACK = 2600.0;

    @Sorter(sort = 8) public static double HOOD_CLOSE = 0.16;
    @Sorter(sort = 9) public static double HOOD_FAR = 0.24;
    @Sorter(sort = 10) public static double HOOD_MIN = 0.12;
    @Sorter(sort = 11) public static double HOOD_MAX = 0.45;
    @Sorter(sort = 12) public static double HOOD_FALLBACK = 0.16;

    // ==================== DISTANCE VALIDATION ====================
    @Sorter(sort = 13) public static double DIST_MIN_VALID = 6.0;
    @Sorter(sort = 14) public static double DIST_MAX_VALID = 180.0;

    // ==================== SMOOTHING PARAMETERS ====================
    @Sorter(sort = 15) public static double RPM_SMOOTHING_ALPHA = 0.15;
    @Sorter(sort = 16) public static double HOOD_SMOOTHING_ALPHA = 0.10;
    @Sorter(sort = 17) public static double DISTANCE_SMOOTHING_ALPHA = 0.2;

    // ==================== HYSTERESIS ====================
    @Sorter(sort = 18) public static double RPM_HYSTERESIS = 25.0;
    @Sorter(sort = 19) public static double HOOD_HYSTERESIS = 0.005;

    // ==================== TIMEOUT ====================
    @Sorter(sort = 20) public static long DETECTION_TIMEOUT_MS = 500;

    // ==================== INTERNAL STATE ====================
    private double lastValidDistance = -1.0;
    private double smoothedDistance = -1.0;
    private double smoothedRpm = RPM_FALLBACK;
    private double smoothedHood = HOOD_FALLBACK;
    private double lastOutputRpm = RPM_FALLBACK;
    private double lastOutputHood = HOOD_FALLBACK;
    private double lastTurretLead = 0.0;
    private long lastValidDetectionTimeMs = 0;
    private boolean hasValidDetection = false;

    // Last ballistic solution (for telemetry)
    private BallisticsCalculator.BallisticSolution lastSolution = null;

    // Robot velocity for lead calculation
    private double robotVx = 0;
    private double robotVy = 0;
    private double targetBearing = 0;

    // ==================== SIMPLE REGRESSION CALCULATIONS ====================

    public static double calculateRpmFromDistance(double distanceInches) {
        double slope = (RPM_FAR - RPM_CLOSE) / (DIST_FAR_INCHES - DIST_CLOSE_INCHES);
        double intercept = RPM_CLOSE - (slope * DIST_CLOSE_INCHES);
        return (slope * distanceInches) + intercept;
    }

    public static double calculateHoodFromDistance(double distanceInches) {
        double slope = (HOOD_FAR - HOOD_CLOSE) / (DIST_FAR_INCHES - DIST_CLOSE_INCHES);
        double intercept = HOOD_CLOSE - (slope * DIST_CLOSE_INCHES);
        return (slope * distanceInches) + intercept;
    }

    public static double getRpmSlope() {
        return (RPM_FAR - RPM_CLOSE) / (DIST_FAR_INCHES - DIST_CLOSE_INCHES);
    }

    public static double getHoodSlope() {
        return (HOOD_FAR - HOOD_CLOSE) / (DIST_FAR_INCHES - DIST_CLOSE_INCHES);
    }

    // ==================== UPDATE METHODS ====================

    /**
     * Set robot velocity for turret lead calculation.
     * Call this each loop with current robot velocity.
     */
    public void setRobotVelocity(double vxInPerS, double vyInPerS, double bearingDeg) {
        this.robotVx = vxInPerS;
        this.robotVy = vyInPerS;
        this.targetBearing = bearingDeg;
    }

    /**
     * Main update method. Calculates RPM, hood position, and turret lead.
     *
     * @param distanceInches Distance to target (-1 if no detection)
     * @param currentTimeMs Current system time
     * @return Target RPM
     */
    public double update(double distanceInches, long currentTimeMs) {
        boolean isValidReading = distanceInches >= DIST_MIN_VALID &&
                distanceInches <= DIST_MAX_VALID;

        if (isValidReading) {
            hasValidDetection = true;
            lastValidDetectionTimeMs = currentTimeMs;
            lastValidDistance = distanceInches;

            // Smooth distance
            if (smoothedDistance < 0) {
                smoothedDistance = distanceInches;
            } else {
                smoothedDistance = (DISTANCE_SMOOTHING_ALPHA * distanceInches) +
                        ((1.0 - DISTANCE_SMOOTHING_ALPHA) * smoothedDistance);
            }

            // Calculate using selected model
            double rawRpm, rawHood;

            if (USE_PHYSICS_MODEL) {
                // Full physics calculation
                lastSolution = BallisticsCalculator.calculate(
                        smoothedDistance, robotVx, robotVy, targetBearing);

                if (lastSolution.valid) {
                    rawRpm = lastSolution.flywheelRpm;
                    rawHood = lastSolution.hoodServoPosition;
                    lastTurretLead = lastSolution.turretLeadAngleDeg;
                } else {
                    // Physics failed, use regression
                    rawRpm = calculateRpmFromDistance(smoothedDistance);
                    rawHood = calculateHoodFromDistance(smoothedDistance);
                    lastTurretLead = 0;
                }
            } else {
                // Simple regression
                rawRpm = calculateRpmFromDistance(smoothedDistance);
                rawHood = calculateHoodFromDistance(smoothedDistance);
                lastTurretLead = 0;
                lastSolution = null;
            }

            // Clamp values
            rawRpm = Math.max(RPM_MIN, Math.min(RPM_MAX, rawRpm));
            rawHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, rawHood));

            // Smooth outputs
            smoothedRpm = (RPM_SMOOTHING_ALPHA * rawRpm) +
                    ((1.0 - RPM_SMOOTHING_ALPHA) * smoothedRpm);
            smoothedHood = (HOOD_SMOOTHING_ALPHA * rawHood) +
                    ((1.0 - HOOD_SMOOTHING_ALPHA) * smoothedHood);

        } else {
            // No valid reading
            long timeSinceLastDetection = currentTimeMs - lastValidDetectionTimeMs;

            if (timeSinceLastDetection > DETECTION_TIMEOUT_MS) {
                hasValidDetection = false;
                smoothedRpm = (RPM_SMOOTHING_ALPHA * RPM_FALLBACK) +
                        ((1.0 - RPM_SMOOTHING_ALPHA) * smoothedRpm);
                smoothedHood = (HOOD_SMOOTHING_ALPHA * HOOD_FALLBACK) +
                        ((1.0 - HOOD_SMOOTHING_ALPHA) * smoothedHood);
                lastTurretLead = 0;
            }
        }

        // Apply hysteresis
        if (Math.abs(smoothedRpm - lastOutputRpm) >= RPM_HYSTERESIS) {
            lastOutputRpm = smoothedRpm;
        }
        if (Math.abs(smoothedHood - lastOutputHood) >= HOOD_HYSTERESIS) {
            lastOutputHood = smoothedHood;
        }

        return lastOutputRpm;
    }

    public double update(boolean hasDetection, double distanceInches, long currentTimeMs) {
        if (!hasDetection) {
            return update(-1.0, currentTimeMs);
        }
        return update(distanceInches, currentTimeMs);
    }

    // ==================== GETTERS ====================

    public double getTargetHoodPosition() {
        return lastOutputHood;
    }

    public double getTurretLeadAngleDeg() {
        return lastTurretLead;
    }

    public double getLastValidDistance() {
        return lastValidDistance;
    }

    public double getSmoothedDistance() {
        return smoothedDistance;
    }

    public double getSmoothedRpm() {
        return smoothedRpm;
    }

    public double getSmoothedHood() {
        return smoothedHood;
    }

    public double getLastOutputRpm() {
        return lastOutputRpm;
    }

    public double getLastOutputHood() {
        return lastOutputHood;
    }

    public boolean hasValidDetection() {
        return hasValidDetection;
    }

    public BallisticsCalculator.BallisticSolution getLastSolution() {
        return lastSolution;
    }

    public void reset() {
        lastValidDistance = -1.0;
        smoothedDistance = -1.0;
        smoothedRpm = RPM_FALLBACK;
        smoothedHood = HOOD_FALLBACK;
        lastOutputRpm = RPM_FALLBACK;
        lastOutputHood = HOOD_FALLBACK;
        lastTurretLead = 0;
        lastValidDetectionTimeMs = 0;
        hasValidDetection = false;
        lastSolution = null;
        robotVx = 0;
        robotVy = 0;
    }

    public void forceSet(double rpm, double hood) {
        smoothedRpm = rpm;
        lastOutputRpm = rpm;
        smoothedHood = hood;
        lastOutputHood = hood;
    }

    public String getStatusString() {
        if (hasValidDetection) {
            if (USE_PHYSICS_MODEL && lastSolution != null) {
                return String.format("PHYSICS | Dist:%.1f\" | RPM:%.0f | Hood:%.3f | Lead:%.1f°",
                        smoothedDistance, lastOutputRpm, lastOutputHood, lastTurretLead);
            } else {
                return String.format("REGRESSION | Dist:%.1f\" | RPM:%.0f | Hood:%.3f",
                        smoothedDistance, lastOutputRpm, lastOutputHood);
            }
        } else {
            return String.format("NO TAG | RPM:%.0f | Hood:%.3f", lastOutputRpm, lastOutputHood);
        }
    }

    public static String getRegressionInfo() {
        if (USE_PHYSICS_MODEL) {
            return "Physics mode: " + BallisticsCalculator.getCalibrationInfo();
        } else {
            return String.format("Regression: RPM=%.3f*d+%.1f | Hood=%.5f*d+%.4f",
                    getRpmSlope(), RPM_CLOSE - getRpmSlope() * DIST_CLOSE_INCHES,
                    getHoodSlope(), HOOD_CLOSE - getHoodSlope() * DIST_CLOSE_INCHES);
        }
    }
}