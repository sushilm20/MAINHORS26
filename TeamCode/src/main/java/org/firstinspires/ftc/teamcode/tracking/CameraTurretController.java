package org.firstinspires.ftc.teamcode.tracking;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Camera-based turret auto-alignment controller.
 *
 * Uses AprilTag detection to automatically rotate the turret toward a target.
 * Based on PD control logic similar to the video tutorial's mecanum drive
 * auto-alignment, but adapted for a single-motor turret.
 *
 * Usage:
 *   1. Create instance with turret motor, AprilTag processor, and optional telemetry
 *   2. Call update() each loop with:
 *      - autoAlignTrigger: true when driver wants auto-alignment (e.g., left trigger pressed)
 *      - manualPower: manual turret control power (e.g., from right stick X)
 *      - targetTagId: which AprilTag ID to track (or -1 for any visible tag)
 */
@Configurable
public class CameraTurretController {

    // Hardware references
    private final DcMotor turretMotor;
    private final AprilTagProcessor aprilTagProcessor;
    private final Telemetry telemetry;

    // Optional encoder reset trigger (e.g., limit switch)
    private BooleanSupplier encoderResetTrigger = null;

    // -------- Turret limits (configurable) --------
    @Sorter(sort = 0)
    public static int TURRET_MIN_POS = -1000;
    @Sorter(sort = 1)
    public static int TURRET_MAX_POS = 1000;

    // -------- PD Controller gains (configurable) --------
    @Sorter(sort = 2)
    public static double KP = 0.02;          // Proportional gain (start low, tune up)
    @Sorter(sort = 3)
    public static double KD = 0.001;         // Derivative gain (start very low)

    // -------- Alignment parameters (configurable) --------
    @Sorter(sort = 4)
    public static double GOAL_BEARING = 0.0;       // Target bearing offset (degrees). 0 = center on tag
    @Sorter(sort = 5)
    public static double ANGLE_TOLERANCE = 1.0;    // Deadband tolerance in degrees
    @Sorter(sort = 6)
    public static double MAX_AUTO_POWER = 0.4;     // Max power during auto-alignment (clamp)

    // -------- Power smoothing (configurable) --------
    @Sorter(sort = 7)
    public static double POWER_SMOOTH_ALPHA = 0.8; // EMA smoothing (0-1, higher = more smoothing)

    // -------- Internal state --------
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double lastAppliedPower = 0.0;
    private boolean wasAutoAlignLastLoop = false;

    // Telemetry/debug values
    private double lastBearing = 0.0;
    private double lastPTerm = 0.0;
    private double lastDTerm = 0.0;
    private double lastCmdPower = 0.0;
    private int lastDetectedTagId = -1;
    private boolean lastAlignedStatus = false;

    // Runtime reference (set via setRuntime or use System time)
    private double runtimeSeconds = 0.0;
    private boolean useExternalRuntime = false;

    /**
     * Constructor with AprilTag processor.
     */
    public CameraTurretController(DcMotor turretMotor, AprilTagProcessor aprilTagProcessor, Telemetry telemetry) {
        this.turretMotor = turretMotor;
        this.aprilTagProcessor = aprilTagProcessor;
        this.telemetry = telemetry;

        resetState();
    }

    /**
     * Constructor without telemetry.
     */
    public CameraTurretController(DcMotor turretMotor, AprilTagProcessor aprilTagProcessor) {
        this(turretMotor, aprilTagProcessor, null);
    }

    /**
     * Set an external runtime source (call this each loop if using OpMode runtime).
     * If not set, System.currentTimeMillis() is used.
     */
    public void setRuntime(double runtimeSeconds) {
        this.runtimeSeconds = runtimeSeconds;
        this.useExternalRuntime = true;
    }

    /**
     * Allow the OpMode to provide a trigger that resets the turret encoder when true.
     */
    public void setEncoderResetTrigger(BooleanSupplier trigger) {
        this.encoderResetTrigger = trigger;
    }

    /**
     * Reset PD state (useful on mode switches).
     */
    public void resetState() {
        lastError = 0.0;
        lastTime = getCurrentTime();
        lastAppliedPower = 0.0;
        wasAutoAlignLastLoop = false;
    }

    /**
     * Get current time in seconds.
     */
    private double getCurrentTime() {
        if (useExternalRuntime) {
            return runtimeSeconds;
        }
        return System.currentTimeMillis() / 1000.0;
    }

    /**
     * Main update method. Call from OpMode loop.
     *
     * @param autoAlignTrigger true when driver wants auto-alignment (e.g., gamepad1.left_trigger > 0.3)
     * @param manualPower      manual turret control (-1 to 1, e.g., gamepad1.right_stick_x)
     * @param targetTagId      which AprilTag ID to track, or -1 for any visible tag
     */
    public void update(boolean autoAlignTrigger, double manualPower, int targetTagId) {
        double now = getCurrentTime();
        int currentPosition = turretMotor.getCurrentPosition();

        // Check encoder reset trigger if configured
        if (encoderResetTrigger != null && encoderResetTrigger.getAsBoolean()) {
            try {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception ignored) {}
        }

        // -------- Sense: Get AprilTag detection --------
        AprilTagDetection targetDetection = null;
        if (aprilTagProcessor != null) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose != null) {
                    if (targetTagId < 0 || detection.id == targetTagId) {
                        targetDetection = detection;
                        break;
                    }
                }
            }
        }

        // -------- Think & Act: Auto-align or manual control --------

        // If auto-align trigger is NOT pressed, use manual control
        if (!autoAlignTrigger) {
            applyManualPower(manualPower, currentPosition);

            // Reset state to prevent derivative spikes when re-entering auto mode
            lastError = 0.0;
            lastTime = now;
            wasAutoAlignLastLoop = false;

            lastDetectedTagId = (targetDetection != null) ? targetDetection.id : -1;
            lastAlignedStatus = false;
            publishTelemetry("MANUAL", currentPosition);
            return;
        }

        // Auto-align is triggered, but check if we have a valid detection
        if (targetDetection == null) {
            // No valid tag detected - fall back to manual or hold position
            applyManualPower(manualPower, currentPosition);

            // Reset state to prevent derivative spikes
            lastError = 0.0;
            lastTime = now;
            wasAutoAlignLastLoop = false;

            lastDetectedTagId = -1;
            lastAlignedStatus = false;
            publishTelemetry("NO_TAG", currentPosition);
            return;
        }

        // Valid tag detected - proceed with auto-alignment PD control
        lastDetectedTagId = targetDetection.id;

        // Get bearing (yaw angle to target in degrees, negative = target is left, positive = right)
        // Note: ftcPose.bearing gives angle from camera centerline to tag
        double bearing = targetDetection.ftcPose.bearing;
        lastBearing = bearing;

        // Calculate error: how far we are from our goal bearing
        // Positive error means we need to rotate in the positive direction
        double error = GOAL_BEARING - bearing;

        // Check if we're already aligned (within tolerance)
        if (Math.abs(error) < ANGLE_TOLERANCE) {
            // On target - stop rotating
            turretMotor.setPower(0.0);
            lastAppliedPower = 0.0;

            // Update state for next loop
            lastError = error;
            lastTime = now;
            wasAutoAlignLastLoop = true;

            lastPTerm = 0.0;
            lastDTerm = 0.0;
            lastCmdPower = 0.0;
            lastAlignedStatus = true;
            publishTelemetry("ALIGNED", currentPosition);
            return;
        }

        // -------- PD Controller calculation --------

        // Calculate delta time
        double dt = now - lastTime;
        if (dt <= 0) dt = 0.02; // Prevent division by zero, assume ~50Hz

        // Proportional term: P = Kp * error
        double pTerm = KP * error;

        // Derivative term: D = Kd * (error - lastError) / dt
        double dTerm = 0.0;
        if (wasAutoAlignLastLoop && dt > 0) {
            // Only apply D term if we were in auto mode last loop (prevent spikes)
            dTerm = KD * (error - lastError) / dt;
        }

        // Combine P and D
        double cmdPower = pTerm + dTerm;

        // Clamp to max auto power
        cmdPower = Range.clip(cmdPower, -MAX_AUTO_POWER, MAX_AUTO_POWER);

        // Apply power smoothing (exponential moving average)
        double smoothedPower = POWER_SMOOTH_ALPHA * lastAppliedPower + (1.0 - POWER_SMOOTH_ALPHA) * cmdPower;

        // Respect hard limits
        if ((currentPosition >= TURRET_MAX_POS && smoothedPower > 0.0) ||
                (currentPosition <= TURRET_MIN_POS && smoothedPower < 0.0)) {
            smoothedPower = 0.0;
        }

        // Apply to motor
        turretMotor.setPower(smoothedPower);

        // Update state for next loop
        lastError = error;
        lastTime = now;
        lastAppliedPower = smoothedPower;
        wasAutoAlignLastLoop = true;

        // Store debug values
        lastPTerm = pTerm;
        lastDTerm = dTerm;
        lastCmdPower = cmdPower;
        lastAlignedStatus = false;

        publishTelemetry("TRACKING", currentPosition);
    }

    /**
     * Simplified update that tracks any visible AprilTag.
     */
    public void update(boolean autoAlignTrigger, double manualPower) {
        update(autoAlignTrigger, manualPower, -1);
    }

    /**
     * Apply manual power with limit protection.
     */
    private void applyManualPower(double manualPower, int currentPosition) {
        double requested = manualPower;

        // Respect hard limits
        if ((currentPosition >= TURRET_MAX_POS && requested > 0.0) ||
                (currentPosition <= TURRET_MIN_POS && requested < 0.0)) {
            requested = 0.0;
        }

        // Clamp to valid range
        requested = Range.clip(requested, -1.0, 1.0);

        turretMotor.setPower(requested);
        lastAppliedPower = requested;

        lastPTerm = 0.0;
        lastDTerm = 0.0;
        lastCmdPower = 0.0;
    }

    /**
     * Disable the controller and stop the motor.
     */
    public void disable() {
        turretMotor.setPower(0.0);
        resetState();
    }

    /**
     * Publish telemetry data (if telemetry is available).
     */
    private void publishTelemetry(String mode, int currentPosition) {
        if (telemetry == null) return;

        telemetry.addData("turretCam.mode", mode);
        telemetry.addData("turretCam.position", currentPosition);
        telemetry.addData("turretCam.tagId", lastDetectedTagId);
        telemetry.addData("turretCam.bearing", String.format("%.2f°", lastBearing));
        telemetry.addData("turretCam.error", String.format("%.2f°", lastError));
        telemetry.addData("turretCam.aligned", lastAlignedStatus);
    }

    // -------- Telemetry getters (for external use) --------

    public double getLastBearing() { return lastBearing; }
    public double getLastError() { return lastError; }
    public double getLastPTerm() { return lastPTerm; }
    public double getLastDTerm() { return lastDTerm; }
    public double getLastAppliedPower() { return lastAppliedPower; }
    public int getLastDetectedTagId() { return lastDetectedTagId; }
    public boolean isAligned() { return lastAlignedStatus; }
    public int getCurrentPosition() { return turretMotor.getCurrentPosition(); }
}