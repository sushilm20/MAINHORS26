package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveController {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    // ===== Drive shaping config (easy to tune) =====
    public static double STICK_DEADBAND = 0.05;
    public static double TRANSLATION_EXPO = 1.6; // 1.0 = linear, >1 more precision near center
    public static double YAW_EXPO = 1.8;
    public static double MAX_TRANSLATION = 1.0;
    public static double MAX_YAW = 0.85; // slight cap helps prevent spinouts

    public DriveController(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    /**
     * Raw drive entrypoint (preserves old behavior contract but now shaped).
     *
     * @param axial forward/backward input (positive forward)
     * @param lateral left/right input (positive right)
     * @param yaw rotation input (positive clockwise)
     * @param speedScale overall scaling 0..1
     */
    public void setDrive(double axial, double lateral, double yaw, double speedScale) {
        // Shape inputs for better low-speed precision
        double a = shapeTranslation(axial);
        double l = shapeTranslation(lateral);
        double y = shapeYaw(yaw);

        setDriveShaped(a, l, y, speedScale);
    }

    /**
     * Use this if caller already shaped inputs.
     */
    public void setDriveShaped(double axial, double lateral, double yaw, double speedScale) {
        // Clamp requested axes
        axial = clip(axial, -MAX_TRANSLATION, MAX_TRANSLATION);
        lateral = clip(lateral, -MAX_TRANSLATION, MAX_TRANSLATION);
        yaw = clip(yaw, -MAX_YAW, MAX_YAW);

        // Mecanum mixing
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize so max magnitude <= 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Apply speed scaling
        speedScale = clip(speedScale, 0.0, 1.0);
        frontLeftPower  *= speedScale;
        frontRightPower *= speedScale;
        backLeftPower   *= speedScale;
        backRightPower  *= speedScale;

        // Write to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /** Stop the drive immediately. */
    public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    // ===== Helpers =====

    private static double shapeTranslation(double x) {
        x = applyDeadband(x, STICK_DEADBAND);
        return expo(x, TRANSLATION_EXPO);
    }

    private static double shapeYaw(double x) {
        x = applyDeadband(x, STICK_DEADBAND);
        return expo(x, YAW_EXPO);
    }

    private static double applyDeadband(double x, double deadband) {
        if (Math.abs(x) <= deadband) return 0.0;
        // Rescale to keep continuity after deadband removal
        double sign = Math.signum(x);
        double mag = (Math.abs(x) - deadband) / (1.0 - deadband);
        return sign * clip(mag, 0.0, 1.0);
    }

    private static double expo(double x, double exponent) {
        double sign = Math.signum(x);
        return sign * Math.pow(Math.abs(x), exponent);
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}