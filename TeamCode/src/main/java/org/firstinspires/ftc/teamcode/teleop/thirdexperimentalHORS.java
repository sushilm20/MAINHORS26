package org.firstinspires.ftc.teamcode.teleop;


/*
  thirdexperimentalHORS.java
  ---------------------------
  - Adds PedroPathing follower for pose tracking only.
  - New FlywheelVersatile computes RPM from distance to blue goal (14,134) using calibration points.
  - Keeps shooter on/off (dpad_down), RPM trim (dpad_left/right), continuous rumble at-target.
  - Removes far/close touchpad mode; RPM auto-adjusts from pose.
  - Gate servo toggle stays on dpad_up.
*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import java.util.Arrays;
import java.util.List;

// subsystem imports (adjust package paths if yours differ)
import org.firstinspires.ftc.teamcode.tracking.TurretController;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelVersatile;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelVersatile.CalibrationPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // ensure this exists in your project

@TeleOp(name="Adaptive HORS", group="Linear OpMode")
public class thirdexperimentalHORS extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;

    // Gate servo
    private Servo gateServo;
    private boolean gateClosed = false;
    private static final double GATE_OPEN = 0.67;//lmao
    private static final double GATE_CLOSED = 0.5;

    // Subsystems
    private TurretController turretController;
    private DriveController driveController;
    private FlywheelController flywheel;
    private FlywheelVersatile flywheelVersatile;

    // PedroPathing follower (for localization)
    private Follower follower;
    private Pose currentPose = new Pose();

    // UI / debounce and other small state
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean xPressedLast = false;
    private boolean yPressedLast = false;

    // hood/claw
    private double leftHoodPosition = 0.12;
    private double rightHoodPosition = 0.12;
    private long lastLeftHoodAdjustMs = 0L;
    private long lastRightHoodAdjustMs = 0L;
    private static final long HOOD_ADJUST_DEBOUNCE_MS = 120L;

    private int clawActionPhase = 0;
    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 500L;

    // For gamepad2 touchpad reset
    private boolean gamepad2TouchpadLast = false;

    // IMUs
    private BNO055IMU imu;            // existing expansion-hub IMU (named "imu" in config)
    private BNO055IMU pinpointImu;    // optional pinpoint IMU (named "pinpoint" in config)
    private BNO055IMU turretImu;      // the IMU actually used by the turret (pinpoint if present otherwise imu)

    // Goal pose (blue goal)
    private static final Pose BLUE_GOAL = new Pose(14, 134, 0);

    @Override
    public void runOpMode() {

        // Hardware map
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        turret = hardwareMap.get(DcMotor.class, "turret");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        // Gate servo (ensure hardware config uses the name "gateServo" or change accordingly)
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // Directions & modes
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU init
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
        } catch (Exception e) {
            imu = null;
        }

        try {
            pinpointImu = hardwareMap.get(BNO055IMU.class, "pinpoint");
        } catch (Exception e) {
            pinpointImu = null;
        }

        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        if (imu != null) {
            try {
                imu.initialize(imuParams);
            } catch (Exception ignored) {}
        }
        if (pinpointImu != null) {
            try {
                pinpointImu.initialize(imuParams);
            } catch (Exception ignored) {}
        }

        // Choose turret IMU: prefer pinpoint if available
        turretImu = (pinpointImu != null) ? pinpointImu : imu;

        // Create subsystem controllers
        turretController = new TurretController(turret, turretImu, telemetry);
        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new FlywheelController(shooter, telemetry);

        // PedroPathing follower for localization (driving still handled by DriveController)
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(40.000, 85.000, Math.toRadians(135)));  // Default starting pose (adjust as needed)
            follower.update();
        } catch (Exception e) {
            follower = null; // fail gracefully
        }

        // FlywheelVersatile setup with calibration samples and RPM bounds
        List<CalibrationPoint> calibrationPoints = Arrays.asList(
                new CalibrationPoint(new Pose(48, 96, 135), 2400),
                new CalibrationPoint(new Pose(60, 125, 135), 2500),
                new CalibrationPoint(new Pose(60, 82, 135), 2600),
                new CalibrationPoint(new Pose(72, 72, 135), 2740),
                new CalibrationPoint(new Pose(72, 120, 167), 2500),
                new CalibrationPoint(new Pose(95, 120, 135), 2900),
                new CalibrationPoint(new Pose(52, 14, 135), 4000)
        );
        flywheelVersatile = new FlywheelVersatile(flywheel, BLUE_GOAL, calibrationPoints, 2400, 4000);

        // initial positions
        clawServo.setPosition(0.63);
        leftHoodServo.setPosition(leftHoodPosition);
        rightHoodPosition = 0.12;
        rightHoodServo.setPosition(rightHoodPosition);
        gateClosed = false;
        gateServo.setPosition(GATE_OPEN);

        String imuUsed = (turretImu == pinpointImu && pinpointImu != null) ? "pinpoint" :
                (turretImu == imu && imu != null) ? "imu (expansion hub)" : "none";
        telemetry.addData("Status", "Initialized (auto RPM)");
        telemetry.addData("Turret IMU", imuUsed);
        telemetry.update();

        // ensure subsystems are ready
        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // Update follower for pose (localization only)
            if (follower != null) {
                try {
                    follower.update();
                    currentPose = follower.getPose();
                } catch (Exception e) {
                    // leave currentPose as last known
                }
            }

            // ------------------------------
            // Touchpad reset (gamepad2)
            // ------------------------------
            boolean gamepad2TouchpadNow = false;
            try { gamepad2TouchpadNow = gamepad2.touchpad; } catch (Throwable t) {
                gamepad2TouchpadNow = (gamepad2.left_stick_button && gamepad2.right_stick_button);
            }
            if (gamepad2TouchpadNow && !gamepad2TouchpadLast) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turretController.captureReferences();
                turretController.resetPidState();

                driveController.stop();

                telemetry.addData("Reset", "IMU heading reference and turret encoder set to zero!");
                telemetry.update();
            }
            gamepad2TouchpadLast = gamepad2TouchpadNow;

            // ------------------------------
            // DRIVE: delegate to DriveController (same as before)
            // ------------------------------
            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x;
            double driveSpeed = 1.0;
            driveController.setDrive(axial, lateral, yaw, driveSpeed);

            // ------------------------------
            // DPAD shooter adjustments (toggle/trim)
            // ------------------------------
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                flywheel.toggleShooterOn();
            }
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) {
                flywheelVersatile.adjustTrim(-10.0);
            }
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) {
                flywheelVersatile.adjustTrim(10.0);
            }
            dpadRightLast = dpadRightNow;

            boolean yNow = gamepad1.y || gamepad2.y;

            // ------------------------------
            // Gate servo toggle on Y for both controllers
            // ------------------------------
            if (yNow && !yPressedLast) {
                gateClosed = !gateClosed;
                gateServo.setPosition(gateClosed ? GATE_CLOSED : GATE_OPEN);
            }
            yPressedLast = yNow;

            // ------------------------------
            // Flywheel target from pose-based model + trim
            // ------------------------------
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);

            double targetRpm = flywheelVersatile.getFinalTargetRPM(currentPose);
            flywheel.setTargetRPM(targetRpm);
            flywheel.update(nowMs, yNow);

            // ------------------------------
            // CONTINUOUS RUMBLE while flywheel within tolerance
            // ------------------------------
            if (flywheel.isAtTarget()) {
                final int RUMBLE_MS = 200;
                try { gamepad1.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
            }

            // ------------------------------
            // TURRET: manual detection and control
            // ------------------------------
            boolean manualNow = false;
            double manualPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.left_stick_x > 0.2) {
                manualNow = true;
                manualPower = 0.5;
            } else if (gamepad1.left_bumper || gamepad2.left_stick_x < -0.2) {
                manualNow = true;
                manualPower = -0.5;
            }
            turretController.update(manualNow, manualPower);

            // ------------------------------
            // INTAKE (compression servos removed)
            // ------------------------------
            boolean leftTriggerNow = (gamepad1.left_trigger > 0.1) || (gamepad2.left_trigger > 0.1);
            if (leftTriggerNow) {
                intakeMotor.setPower(-1.0);
            } else if ((gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                intakeMotor.setPower(0.5);
            } else {
                intakeMotor.setPower(0.0);
            }

            // CLAW toggle
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawServo.setPosition(0.2);
                clawActionPhase = 1;
                clawActionStartMs = nowMs;
            }
            xPressedLast = xNow;
            if (clawActionPhase == 1 && nowMs >= clawActionStartMs + CLAW_CLOSE_MS) {
                clawServo.setPosition(0.63);
                clawActionPhase = 0;
            }

            // Hood adjustments
            if (gamepad1.a && nowMs - lastLeftHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastLeftHoodAdjustMs = nowMs;
                leftHoodPosition = Math.min(0.45, leftHoodPosition + 0.025);
                leftHoodServo.setPosition(leftHoodPosition);
            }
            if (gamepad1.b && nowMs - lastLeftHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastLeftHoodAdjustMs = nowMs;
                leftHoodPosition = Math.max(0.12, leftHoodPosition - 0.025);
                leftHoodServo.setPosition(leftHoodPosition);
            }

            if (gamepad2.right_stick_y < -0.2 && nowMs - lastRightHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastRightHoodAdjustMs = nowMs;
                rightHoodPosition = Math.min(0.45, rightHoodPosition + 0.01);
                rightHoodServo.setPosition(rightHoodPosition);
            } else if (gamepad2.right_stick_y > 0.2 && nowMs - lastRightHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastRightHoodAdjustMs = nowMs;
                rightHoodPosition = Math.max(0.12, rightHoodPosition - 0.01);
                rightHoodServo.setPosition(rightHoodPosition);
            }

            // Summary telemetry
            String imuUsedNow = (turretImu == pinpointImu && pinpointImu != null) ? "pinpoint" :
                    (turretImu == imu && imu != null) ? "imu (exp hub)" : "none";

            telemetry.addData("Pose", currentPose != null ?
                    String.format("(%.1f, %.1f, %.1f°)", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()))
                    : "N/A");
            telemetry.addData("Dist->Goal", currentPose != null ?
                    String.format("%.1f", flywheelVersatile.getLastDistance()) : "N/A");
//            telemetry.addData("Base RPM (model)", String.format("%.1f", flywheelVersatile.getLastBaseRpm()));
//            telemetry.addData("Trim RPM", String.format("%.1f", flywheelVersatile.getTrimRpm()));
            telemetry.addData("Target RPM", String.format("%.1f", targetRpm));
            telemetry.addData("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
//            telemetry.addData("Fly AtTarget", flywheel.isAtTarget());
//            telemetry.addData("Gate", gateClosed ? "CLOSED" : "OPEN");
//            telemetry.addData("Gate Pos", gateServo.getPosition());
//            telemetry.addData("Turret Enc", turret.getCurrentPosition());
//            telemetry.addData("Turret Power (applied)", turretController.getLastAppliedPower());
//            telemetry.addData("Turret IMU Used", imuUsedNow);

            telemetry.update();
        }
    }

    private void headingReferenceReset() {
        // turretController.captureReferences() handles the turret mapping if needed.
    }
}