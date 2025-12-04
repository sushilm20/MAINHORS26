package org.firstinspires.ftc.teamcode.tracking;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * TurretTracker class handles the logic for tracking an AprilTag
 * and centering the turret based on the tag's X position.
 */
public class TurretTracker {
    private DcMotor turretMotor;
    private int targetAprilTagId;

    // Turret limits
    private static final int TURRET_MIN_LIMIT = -400;
    private static final int TURRET_MAX_LIMIT = 400;

    // Camera center tolerance (pixels) - adjust based on camera resolution
    private static final double CENTER_TOLERANCE = 67.0;

    // Turret movement speed
    private static final double TURRET_SPEED = 0.3;

    // Camera center X (assuming 640x480 resolution, center is at 320)
    private static final double CAMERA_CENTER_X = 320.0;

    /**
     * Constructor for TurretTracker
     * @param turretMotor The motor that controls the turret
     * @param targetAprilTagId The ID of the AprilTag to track
     */
    public TurretTracker(DcMotor turretMotor, int targetAprilTagId) {
        this.turretMotor = turretMotor;
        this.targetAprilTagId = targetAprilTagId;

        // Configure turret motor
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Update turret position based on AprilTag detection
     * @param detection The AprilTag detection (can be null if not detected)
     * @return Status string describing the current state
     */
    public String updateTurret(AprilTagDetection detection) {
        int currentPosition = turretMotor.getCurrentPosition();

        // If no detection or wrong tag, stop the turret
        if (detection == null || detection.id != targetAprilTagId) {
            turretMotor.setPower(0.0);
            return "Searching for AprilTag " + targetAprilTagId;
        }

        // Get the X position of the detected tag
        double tagX = detection.center.x;
        double xError = tagX - CAMERA_CENTER_X;

        // If tag is centered (within tolerance), stop
        if (Math.abs(xError) < CENTER_TOLERANCE) {
            turretMotor.setPower(0.0);
            return "Centered on AprilTag " + targetAprilTagId;
        }

        // Determine direction and check limits
        double turretPower = 0.0;

        if (xError > 0) {
            // Tag is to the right, move turret right (positive direction)
            if (currentPosition < TURRET_MAX_LIMIT) {
                turretPower = TURRET_SPEED;
            }
        } else {
            // Tag is to the left, move turret left (negative direction)
            if (currentPosition > TURRET_MIN_LIMIT) {
                turretPower = -TURRET_SPEED;
            }
        }

        turretMotor.setPower(turretPower);

        return String.format("Tracking: X error = %.1f, Power = %.2f", xError, turretPower);
    }

    /**
     * Get the current turret position
     * @return Current encoder position
     */
    public int getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    /**
     * Check if turret is within limits
     * @return True if within limits, false otherwise
     */
    public boolean isWithinLimits() {
        int position = turretMotor.getCurrentPosition();
        return position >= TURRET_MIN_LIMIT && position <= TURRET_MAX_LIMIT;
    }

    /**
     * Stop the turret motor
     */
    public void stop() {
        turretMotor.setPower(0.0);
    }
}