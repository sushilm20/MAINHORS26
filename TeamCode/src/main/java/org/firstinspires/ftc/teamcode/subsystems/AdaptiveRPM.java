package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * AdaptiveRPM - Calculates target flywheel RPM and hood position based on distance to AprilTag.
 *
 * Uses linear regression between two calibration points:
 * - CLOSE: 24 inches -> 2600 RPM -> HOOD_CLOSE position
 * - FAR: 120 inches -> 3000 RPM -> HOOD_FAR position
 *
 * Both RPM and hood position scale linearly with distance.
 */
@Configurable
public class AdaptiveRPM {

    // ==================== RPM CALIBRATION ====================
    @Sorter(sort = 0) public static double DIST_CLOSE_INCHES = 24.0;    // Close calibration distance
    @Sorter(sort = 1) public static double RPM_CLOSE = 2600.0;          // RPM at close distance
    @Sorter(sort = 2) public static double DIST_FAR_INCHES = 120.0;     // Far calibration distance
    @Sorter(sort = 3) public static double RPM_FAR = 3000.0;            // RPM at far distance

    // --- RPM limits ---
    @Sorter(sort = 4) public static double RPM_MIN = 2400.0;            // Minimum allowed RPM
    @Sorter(sort = 5) public static double RPM_MAX = 3500.0;            // Maximum allowed RPM

    // --- Fallback RPM when no tag detected ---
    @Sorter(sort = 6) public static double RPM_FALLBACK = 2600.0;       // Default to close RPM

    // ==================== HOOD CALIBRATION ====================
    @Sorter(sort = 7) public static double HOOD_CLOSE = 0.16;           // Hood position at close distance
    @Sorter(sort = 8) public static double HOOD_FAR = 0.24;             // Hood position at far distance
    @Sorter(sort = 9) public static double HOOD_MIN = 0.12;             // Absolute minimum hood position
    @Sorter(sort = 10) public static double HOOD_MAX = 0.45;            // Absolute maximum hood position
    @Sorter(sort = 11) public static double HOOD_FALLBACK = 0.16;       // Fallback hood position (close)

    // ==================== DISTANCE VALIDATION ====================
    @Sorter(sort = 12) public static double DIST_MIN_VALID = 6.0;       // Minimum valid distance (inches)
    @Sorter(sort = 13) public static double DIST_MAX_VALID = 180.0;     // Maximum valid distance (inches)

    // ==================== SMOOTHING PARAMETERS ====================
    @Sorter(sort = 14) public static double RPM_SMOOTHING_ALPHA = 0.15;      // Lower = smoother (0-1)
    @Sorter(sort = 15) public static double HOOD_SMOOTHING_ALPHA = 0.10;     // Hood smoothing (slower)
    @Sorter(sort = 16) public static double DISTANCE_SMOOTHING_ALPHA = 0.2;  // Distance filter

    // ==================== HYSTERESIS ====================
    @Sorter(sort = 17) public static double RPM_HYSTERESIS = 25.0;      // Min RPM change to update
    @Sorter(sort = 18) public static double HOOD_HYSTERESIS = 0.005;    // Min hood change to update

    // ==================== TIMEOUT ====================
    @Sorter(sort = 19) public static long DETECTION_TIMEOUT_MS = 500;   // Time before fallback

    // ==================== INTERNAL STATE ====================
    private double lastValidDistance = -1.0;
    private double smoothedDistance = -1.0;
    private double smoothedRpm = RPM_FALLBACK;
    private double smoothedHood = HOOD_FALLBACK;
    private double lastOutputRpm = RPM_FALLBACK;
    private double lastOutputHood = HOOD_FALLBACK;
    private long lastValidDetectionTimeMs = 0;
    private boolean hasValidDetection = false;

    // ==================== RPM CALCULATIONS ====================

    /**
     * Calculate RPM based on distance using linear regression.
     *
     * @param distanceInches Distance to AprilTag in inches
     * @return Calculated RPM (unclamped, unsmoothed)
     */
    public static double calculateRpmFromDistance(double distanceInches) {
        double slope = getRpmSlope();
        double intercept = getRpmIntercept();
        return (slope * distanceInches) + intercept;
    }

    /**
     * Get the slope of the RPM-distance relationship.
     */
    public static double getRpmSlope() {
        return (RPM_FAR - RPM_CLOSE) / (DIST_FAR_INCHES - DIST_CLOSE_INCHES);
    }

    /**
     * Get the intercept of the RPM-distance relationship.
     */
    public static double getRpmIntercept() {
        double slope = getRpmSlope();
        return RPM_CLOSE - (slope * DIST_CLOSE_INCHES);
    }

    // ==================== HOOD CALCULATIONS ====================

    /**
     * Calculate hood position based on distance using linear regression.
     *
     * @param distanceInches Distance to AprilTag in inches
     * @return Calculated hood position (unclamped, unsmoothed)
     */
    public static double calculateHoodFromDistance(double distanceInches) {
        double slope = getHoodSlope();
        double intercept = getHoodIntercept();
        return (slope * distanceInches) + intercept;
    }

    /**
     * Calculate hood position based on RPM using linear regression.
     * Maps RPM linearly to hood position.
     *
     * @param rpm Target RPM
     * @return Calculated hood position
     */
    public static double calculateHoodFromRpm(double rpm) {
        // Linear interpolation: hood = HOOD_CLOSE + (rpm - RPM_CLOSE) * hoodPerRpm
        double rpmRange = RPM_FAR - RPM_CLOSE;
        double hoodRange = HOOD_FAR - HOOD_CLOSE;

        if (Math.abs(rpmRange) < 1e-6) {
            return HOOD_CLOSE; // Avoid division by zero
        }

        double hoodPerRpm = hoodRange / rpmRange;
        double hood = HOOD_CLOSE + ((rpm - RPM_CLOSE) * hoodPerRpm);

        return hood;
    }

    /**
     * Get the slope of the hood-distance relationship.
     */
    public static double getHoodSlope() {
        return (HOOD_FAR - HOOD_CLOSE) / (DIST_FAR_INCHES - DIST_CLOSE_INCHES);
    }

    /**
     * Get the intercept of the hood-distance relationship.
     */
    public static double getHoodIntercept() {
        double slope = getHoodSlope();
        return HOOD_CLOSE - (slope * DIST_CLOSE_INCHES);
    }

    // ==================== UPDATE METHODS ====================

    /**
     * Update with a new distance reading and get the target RPM and hood position.
     * Handles smoothing, clamping, and fallback logic.
     *
     * @param distanceInches Current distance reading (-1 or invalid if no detection)
     * @param currentTimeMs Current system time in milliseconds
     * @return Target RPM to set on flywheel
     */
    public double update(double distanceInches, long currentTimeMs) {
        boolean isValidReading = distanceInches >= DIST_MIN_VALID &&
                distanceInches <= DIST_MAX_VALID;

        if (isValidReading) {
            // Update detection state
            hasValidDetection = true;
            lastValidDetectionTimeMs = currentTimeMs;
            lastValidDistance = distanceInches;

            // Smooth the distance reading
            if (smoothedDistance < 0) {
                smoothedDistance = distanceInches;
            } else {
                smoothedDistance = (DISTANCE_SMOOTHING_ALPHA * distanceInches) +
                        ((1.0 - DISTANCE_SMOOTHING_ALPHA) * smoothedDistance);
            }

            // Calculate raw RPM from smoothed distance
            double rawRpm = calculateRpmFromDistance(smoothedDistance);
            rawRpm = Math.max(RPM_MIN, Math.min(RPM_MAX, rawRpm));

            // Calculate raw hood from smoothed distance
            double rawHood = calculateHoodFromDistance(smoothedDistance);
            rawHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, rawHood));

            // Smooth both outputs
            smoothedRpm = (RPM_SMOOTHING_ALPHA * rawRpm) +
                    ((1.0 - RPM_SMOOTHING_ALPHA) * smoothedRpm);
            smoothedHood = (HOOD_SMOOTHING_ALPHA * rawHood) +
                    ((1.0 - HOOD_SMOOTHING_ALPHA) * smoothedHood);

        } else {
            // No valid reading - check timeout
            long timeSinceLastDetection = currentTimeMs - lastValidDetectionTimeMs;

            if (timeSinceLastDetection > DETECTION_TIMEOUT_MS) {
                hasValidDetection = false;
                // Smoothly transition to fallback values
                smoothedRpm = (RPM_SMOOTHING_ALPHA * RPM_FALLBACK) +
                        ((1.0 - RPM_SMOOTHING_ALPHA) * smoothedRpm);
                smoothedHood = (HOOD_SMOOTHING_ALPHA * HOOD_FALLBACK) +
                        ((1.0 - HOOD_SMOOTHING_ALPHA) * smoothedHood);
            }
        }

        // Apply hysteresis to RPM
        if (Math.abs(smoothedRpm - lastOutputRpm) >= RPM_HYSTERESIS) {
            lastOutputRpm = smoothedRpm;
        }

        // Apply hysteresis to hood
        if (Math.abs(smoothedHood - lastOutputHood) >= HOOD_HYSTERESIS) {
            lastOutputHood = smoothedHood;
        }

        return lastOutputRpm;
    }

    /**
     * Update with detection data directly.
     */
    public double update(boolean hasDetection, double distanceInches, long currentTimeMs) {
        if (!hasDetection) {
            return update(-1.0, currentTimeMs);
        }
        return update(distanceInches, currentTimeMs);
    }

    /**
     * Get the current target hood position.
     * Call this after update() to get the smoothed hood value.
     */
    public double getTargetHoodPosition() {
        return lastOutputHood;
    }

    /**
     * Get the raw (unsmoothed) hood position for current distance.
     */
    public double getRawHoodPosition() {
        if (smoothedDistance < 0) {
            return HOOD_FALLBACK;
        }
        double rawHood = calculateHoodFromDistance(smoothedDistance);
        return Math.max(HOOD_MIN, Math.min(HOOD_MAX, rawHood));
    }

    /**
     * Reset internal state.
     */
    public void reset() {
        lastValidDistance = -1.0;
        smoothedDistance = -1.0;
        smoothedRpm = RPM_FALLBACK;
        smoothedHood = HOOD_FALLBACK;
        lastOutputRpm = RPM_FALLBACK;
        lastOutputHood = HOOD_FALLBACK;
        lastValidDetectionTimeMs = 0;
        hasValidDetection = false;
    }

    /**
     * Force set both RPM and hood (bypasses adaptive calculation).
     */
    public void forceSet(double rpm, double hood) {
        smoothedRpm = rpm;
        lastOutputRpm = rpm;
        smoothedHood = hood;
        lastOutputHood = hood;
    }

    /**
     * Force set RPM only.
     */
    public void forceSetRpm(double rpm) {
        smoothedRpm = rpm;
        lastOutputRpm = rpm;
        // Also update hood based on RPM
        double hood = calculateHoodFromRpm(rpm);
        hood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hood));
        smoothedHood = hood;
        lastOutputHood = hood;
    }

    // ==================== GETTERS ====================

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

    /**
     * Get a formatted status string for telemetry.
     */
    public String getStatusString() {
        if (hasValidDetection) {
            return String.format("TRACKING | Dist: %.1f in | RPM: %.0f | Hood: %.3f",
                    smoothedDistance, lastOutputRpm, lastOutputHood);
        } else {
            return String.format("NO TAG | RPM: %.0f | Hood: %.3f",
                    lastOutputRpm, lastOutputHood);
        }
    }

    /**
     * Get regression info string for telemetry.
     */
    public static String getRegressionInfo() {
        return String.format("RPM: %.3f*dist + %.1f | Hood: %.5f*dist + %.4f",
                getRpmSlope(), getRpmIntercept(),
                getHoodSlope(), getHoodIntercept());
    }

    // ==================== STATIC PREVIEW METHODS ====================

    /**
     * Preview what RPM would be for a given distance (no state change).
     */
    public static double previewRpmForDistance(double distanceInches) {
        double rpm = calculateRpmFromDistance(distanceInches);
        return Math.max(RPM_MIN, Math.min(RPM_MAX, rpm));
    }

    /**
     * Preview what hood would be for a given distance (no state change).
     */
    public static double previewHoodForDistance(double distanceInches) {
        double hood = calculateHoodFromDistance(distanceInches);
        return Math.max(HOOD_MIN, Math.min(HOOD_MAX, hood));
    }

    /**
     * Preview what hood would be for a given RPM (no state change).
     */
    public static double previewHoodForRpm(double rpm) {
        double hood = calculateHoodFromRpm(rpm);
        return Math.max(HOOD_MIN, Math.min(HOOD_MAX, hood));
    }
}