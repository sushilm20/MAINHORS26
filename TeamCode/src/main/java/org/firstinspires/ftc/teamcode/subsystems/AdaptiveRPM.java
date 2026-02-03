package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * AdaptiveRPM - Calculates target flywheel RPM based on distance to AprilTag.
 *
 * Uses linear regression between two calibration points:
 * - CLOSE: 24 inches -> 2600 RPM
 * - FAR: 120 inches -> 3000 RPM
 *
 * Formula: RPM = slope * distance + intercept
 * Where slope = (RPM_FAR - RPM_CLOSE) / (DIST_FAR - DIST_CLOSE)
 */
@Configurable
public class AdaptiveRPM {

    // --- Calibration points (Panels-configurable) ---
    @Sorter(sort = 0) public static double DIST_CLOSE_INCHES = 24.0;    // Close calibration distance
    @Sorter(sort = 1) public static double RPM_CLOSE = 2600.0;          // RPM at close distance
    @Sorter(sort = 2) public static double DIST_FAR_INCHES = 120.0;     // Far calibration distance
    @Sorter(sort = 3) public static double RPM_FAR = 3000.0;            // RPM at far distance

    // --- RPM limits ---
    @Sorter(sort = 4) public static double RPM_MIN = 2400.0;            // Minimum allowed RPM
    @Sorter(sort = 5) public static double RPM_MAX = 3500.0;            // Maximum allowed RPM

    // --- Fallback RPM when no tag detected ---
    @Sorter(sort = 6) public static double RPM_FALLBACK = 2600.0;       // Default to close RPM

    // --- Distance validity range ---
    @Sorter(sort = 7) public static double DIST_MIN_VALID = 6.0;        // Minimum valid distance (inches)
    @Sorter(sort = 8) public static double DIST_MAX_VALID = 180.0;      // Maximum valid distance (inches)

    // --- Smoothing parameters ---
    @Sorter(sort = 9) public static double RPM_SMOOTHING_ALPHA = 0.15;  // Lower = smoother (0-1)
    @Sorter(sort = 10) public static double DISTANCE_SMOOTHING_ALPHA = 0.2; // Distance filter

    // --- Hysteresis to prevent oscillation ---
    @Sorter(sort = 11) public static double RPM_HYSTERESIS = 25.0;      // Min change to update

    // --- Internal state ---
    private double lastValidDistance = -1.0;
    private double smoothedDistance = -1.0;
    private double smoothedRpm = RPM_FALLBACK;
    private double lastOutputRpm = RPM_FALLBACK;
    private long lastValidDetectionTimeMs = 0;
    private boolean hasValidDetection = false;

    // --- Timeout for falling back to default RPM ---
    @Sorter(sort = 12) public static long DETECTION_TIMEOUT_MS = 500;   // Time before fallback

    /**
     * Calculate RPM based on distance using linear regression.
     *
     * @param distanceInches Distance to AprilTag in inches
     * @return Calculated RPM (unclamped, unsmoothed)
     */
    public static double calculateRpmFromDistance(double distanceInches) {
        // Linear regression: y = mx + b
        // m = (y2 - y1) / (x2 - x1)
        // b = y1 - m * x1

        double slope = (RPM_FAR - RPM_CLOSE) / (DIST_FAR_INCHES - DIST_CLOSE_INCHES);
        double intercept = RPM_CLOSE - (slope * DIST_CLOSE_INCHES);

        return (slope * distanceInches) + intercept;
    }

    /**
     * Get the slope of the RPM-distance relationship.
     * Useful for telemetry/debugging.
     */
    public static double getSlope() {
        return (RPM_FAR - RPM_CLOSE) / (DIST_FAR_INCHES - DIST_CLOSE_INCHES);
    }

    /**
     * Get the intercept of the RPM-distance relationship.
     */
    public static double getIntercept() {
        double slope = getSlope();
        return RPM_CLOSE - (slope * DIST_CLOSE_INCHES);
    }

    /**
     * Update with a new distance reading and get the target RPM.
     * Handles smoothing, clamping, and fallback logic.
     *
     * @param distanceInches Current distance reading (-1 or invalid if no detection)
     * @param currentTimeMs Current system time in milliseconds
     * @return Target RPM to set on flywheel
     */
    public double update(double distanceInches, long currentTimeMs) {
        // Check if we have a valid distance reading
        boolean isValidReading = distanceInches >= DIST_MIN_VALID &&
                distanceInches <= DIST_MAX_VALID;

        if (isValidReading) {
            // Update detection state
            hasValidDetection = true;
            lastValidDetectionTimeMs = currentTimeMs;
            lastValidDistance = distanceInches;

            // Smooth the distance reading
            if (smoothedDistance < 0) {
                smoothedDistance = distanceInches; // First reading
            } else {
                smoothedDistance = (DISTANCE_SMOOTHING_ALPHA * distanceInches) +
                        ((1.0 - DISTANCE_SMOOTHING_ALPHA) * smoothedDistance);
            }

            // Calculate raw RPM from smoothed distance
            double rawRpm = calculateRpmFromDistance(smoothedDistance);

            // Clamp to valid range
            rawRpm = Math.max(RPM_MIN, Math.min(RPM_MAX, rawRpm));

            // Smooth the RPM output
            smoothedRpm = (RPM_SMOOTHING_ALPHA * rawRpm) +
                    ((1.0 - RPM_SMOOTHING_ALPHA) * smoothedRpm);

        } else {
            // No valid reading - check timeout
            long timeSinceLastDetection = currentTimeMs - lastValidDetectionTimeMs;

            if (timeSinceLastDetection > DETECTION_TIMEOUT_MS) {
                // Timeout exceeded - fall back to default RPM
                hasValidDetection = false;
                smoothedRpm = (RPM_SMOOTHING_ALPHA * RPM_FALLBACK) +
                        ((1.0 - RPM_SMOOTHING_ALPHA) * smoothedRpm);
            }
            // Otherwise, keep using the last smoothed RPM (coast through brief dropouts)
        }

        // Apply hysteresis to prevent small oscillations
        if (Math.abs(smoothedRpm - lastOutputRpm) >= RPM_HYSTERESIS) {
            lastOutputRpm = smoothedRpm;
        }

        return lastOutputRpm;
    }

    /**
     * Update with detection data directly.
     *
     * @param hasDetection Whether an AprilTag was detected
     * @param distanceInches Distance to tag (only used if hasDetection is true)
     * @param currentTimeMs Current system time
     * @return Target RPM
     */
    public double update(boolean hasDetection, double distanceInches, long currentTimeMs) {
        if (!hasDetection) {
            return update(-1.0, currentTimeMs);
        }
        return update(distanceInches, currentTimeMs);
    }

    /**
     * Reset internal state (call when re-initializing or after long pause).
     */
    public void reset() {
        lastValidDistance = -1.0;
        smoothedDistance = -1.0;
        smoothedRpm = RPM_FALLBACK;
        lastOutputRpm = RPM_FALLBACK;
        lastValidDetectionTimeMs = 0;
        hasValidDetection = false;
    }

    /**
     * Force set the RPM (bypasses adaptive calculation).
     * Useful for manual override.
     */
    public void forceSetRpm(double rpm) {
        smoothedRpm = rpm;
        lastOutputRpm = rpm;
    }

    // --- Getters for telemetry ---

    public double getLastValidDistance() {
        return lastValidDistance;
    }

    public double getSmoothedDistance() {
        return smoothedDistance;
    }

    public double getSmoothedRpm() {
        return smoothedRpm;
    }

    public double getLastOutputRpm() {
        return lastOutputRpm;
    }

    public boolean hasValidDetection() {
        return hasValidDetection;
    }

    /**
     * Get a formatted status string for telemetry.
     */
    public String getStatusString() {
        if (hasValidDetection) {
            return String.format("TRACKING | Dist: %.1f in | RPM: %.0f",
                    smoothedDistance, lastOutputRpm);
        } else {
            return String.format("NO TAG | Fallback RPM: %.0f", lastOutputRpm);
        }
    }

    /**
     * Static utility: Calculate what RPM would be for a given distance.
     * Does not affect internal state.
     */
    public static double previewRpmForDistance(double distanceInches) {
        double rpm = calculateRpmFromDistance(distanceInches);
        return Math.max(RPM_MIN, Math.min(RPM_MAX, rpm));
    }
}