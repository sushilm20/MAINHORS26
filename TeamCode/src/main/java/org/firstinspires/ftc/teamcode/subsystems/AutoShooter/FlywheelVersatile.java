package org.firstinspires.ftc.teamcode.subsystems.AutoShooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

/**
 * FlywheelVersatile — Pose-adaptive RPM wrapper around {@link FlywheelController}.
 *
 * Uses a linear regression (computed from {@link ShooterCalibration} points)
 * to map the robot's distance-to-goal into a target RPM.  A manual trim
 * offset is added on top and the result is clamped to [minRpm, maxRpm].
 *
 * <p><b>Usage in TeleOp:</b></p>
 * <pre>
 *   ShooterCalibration cal = new ShooterCalibration();
 *   cal.computeRegression();
 *   flywheelVersatile = new FlywheelVersatile(flywheel, BLUE_GOAL, cal, 2400, 4000);
 *
 *   // each loop:
 *   double targetRpm = flywheelVersatile.getFinalTargetRPM(currentPose);
 *   flywheel.setTargetRPM(targetRpm);
 *   flywheel.update();
 * </pre>
 *
 * This class does NOT own the PIDF loop — it only computes the target RPM.
 * The existing {@link FlywheelController} handles all motor control.
 */
@Configurable
public class FlywheelVersatile {

    // ── References ────────────────────────────────────────────
    private final FlywheelController flywheel;
    private final Pose goalPose;
    private final ShooterCalibration calibration;

    // ── Clamp bounds ──────────────────────────────────────────
    @Sorter(sort = 0) public static double minRpm = 2300.0;
    @Sorter(sort = 1) public static double maxRpm = 4000.0;

    // ── Trim (manual offset added on top of regression) ──────
    @Sorter(sort = 2) public static double trimRpm = 0.0;

    // ── Telemetry cache ───────────────────────────────────────
    private double lastDistance = 0.0;
    private double lastBaseRpm = 0.0;

    // =========================================================================
    //  Constructors
    // =========================================================================

    /**
     * Primary constructor.
     *
     * @param flywheel    the underlying FlywheelController (motor control)
     * @param goalPose    field-coordinate Pose of the target (only x, y used)
     * @param calibration ShooterCalibration that holds the points + regression
     * @param minRpm      hard-floor RPM clamp
     * @param maxRpm      hard-ceiling RPM clamp
     */
    public FlywheelVersatile(FlywheelController flywheel,
                             Pose goalPose,
                             ShooterCalibration calibration,
                             double minRpm,
                             double maxRpm) {
        this.flywheel    = flywheel;
        this.goalPose    = goalPose;
        this.calibration = calibration;
        FlywheelVersatile.minRpm = minRpm;
        FlywheelVersatile.maxRpm = maxRpm;
    }

    /** Convenience: uses the bounds already stored in the static fields. */
    public FlywheelVersatile(FlywheelController flywheel,
                             Pose goalPose,
                             ShooterCalibration calibration) {
        this.flywheel    = flywheel;
        this.goalPose    = goalPose;
        this.calibration = calibration;
    }

    // =========================================================================
    //  Core: compute RPM from a live Pose
    // =========================================================================

    /**
     * Returns the target RPM for the current robot pose, including trim.
     * Clamped to [{@link #minRpm}, {@link #maxRpm}].
     *
     * @param robotPose the robot's current field pose (from Pedro follower)
     * @return the RPM the flywheel should spin at
     */
    public double getFinalTargetRPM(Pose robotPose) {
        if (robotPose == null) {
            return lastBaseRpm + trimRpm;  // fallback: last known
        }

        double dx = robotPose.getX() - goalPose.getX();
        double dy = robotPose.getY() - goalPose.getY();
        double distance = Math.hypot(dx, dy);

        lastDistance = distance;
        lastBaseRpm = calibration.rpmForDistance(distance);

        double finalRpm = lastBaseRpm + trimRpm;
        return Math.max(minRpm, Math.min(maxRpm, finalRpm));
    }

    // =========================================================================
    //  Trim control  (dpad left / right in TeleOp)
    // =========================================================================

    /** Add a delta to the manual trim offset. */
    public void adjustTrim(double delta) {
        trimRpm += delta;
    }

    /** Reset the trim back to zero. */
    public void resetTrim() {
        trimRpm = 0.0;
    }

    // =========================================================================
    //  Re-calibrate live  (after Panels changes)
    // =========================================================================

    /** Rebuilds the regression from the (possibly changed) calibration fields. */
    public void recalibrate() {
        calibration.computeRegression();
    }

    // =========================================================================
    //  Getters for telemetry
    // =========================================================================

    public double getLastDistance() { return lastDistance; }
    public double getLastBaseRpm() { return lastBaseRpm; }
    public double getTrimRpm()     { return trimRpm; }
    public double getMinRpm()      { return minRpm; }
    public double getMaxRpm()      { return maxRpm; }

    /** Regression slope (RPM per unit distance). */
    public double getSlope()       { return calibration.getSlope(); }
    /** Regression y-intercept. */
    public double getIntercept()   { return calibration.getIntercept(); }

    /** Returns the underlying FlywheelController. */
    public FlywheelController getFlywheel() { return flywheel; }
}