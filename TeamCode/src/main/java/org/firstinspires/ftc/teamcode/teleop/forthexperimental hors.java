package org.firstinspires.ftc.teamcode.teleop;

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

import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelVersatile;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelVersatile.CalibrationPoint;
import org.firstinspires.ftc.teamcode.subsystems.TurretAutoAim;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Turret and DynamicHORS", group="Linear OpMode")
public class forthexperimentalHORS experimentalHORS extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, turret, intakeMotor;
    private Servo clawServo, leftCompressionServo, rightCompressionServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;

    private boolean dpadUpLast = false;
    private boolean gateClosed = false;
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.5;

    private DriveController driveController;
    private Flywheel flywheel;
    private FlywheelVersatile flywheelVersatile;
    private TurretAutoAim turretAutoAim;

    private Follower follower;
    private Pose currentPose = new Pose();

    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean xPressedLast = false;

    private double leftHoodPosition = 0.12;
    private double rightHoodPosition = 0.12;
    private long lastLeftHoodAdjustMs = 0L;
    private long lastRightHoodAdjustMs = 0L;
    private static final long HOOD_ADJUST_DEBOUNCE_MS = 120L;

    private int clawActionPhase = 0;
    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 500L;

    private boolean gamepad2TouchpadLast = false;

    private BNO055IMU imu;
    private BNO055IMU pinpointImu;
    private BNO055IMU turretImu;

    private static final Pose BLUE_GOAL = new Pose(14, 134, 0);

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        turret = hardwareMap.get(DcMotor.class, "turret");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftCompressionServo = hardwareMap.get(Servo.class, "leftCompressionServo");
        rightCompressionServo = hardwareMap.get(Servo.class, "rightCompressionServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU init (for completeness—turret aiming here does not depend on IMU)
        try { imu = hardwareMap.get(BNO055IMU.class, "imu"); } catch (Exception e) { imu = null; }
        try { pinpointImu = hardwareMap.get(BNO055IMU.class, "pinpoint"); } catch (Exception e) { pinpointImu = null; }
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        if (imu != null) try { imu.initialize(imuParams); } catch (Exception ignored) {}
        if (pinpointImu != null) try { pinpointImu.initialize(imuParams); } catch (Exception ignored) {}
        turretImu = (pinpointImu != null) ? pinpointImu : imu;

        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new Flywheel(shooter, telemetry);

        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(72, 72, 0));
            follower.update();
            currentPose = follower.getPose();
        } catch (Exception e) {
            follower = null;
        }

        List<CalibrationPoint> calibrationPoints = Arrays.asList(
                new CalibrationPoint(new Pose(48, 96, 0), 90.0),
                new CalibrationPoint(new Pose(60, 125, 0), 95.0),
                new CalibrationPoint(new Pose(60, 82, 0), 100.0),
                new CalibrationPoint(new Pose(72, 72, 0), 110.0),
                new CalibrationPoint(new Pose(52, 14, 0), 140.0)
        );
        flywheelVersatile = new FlywheelVersatile(flywheel, BLUE_GOAL, calibrationPoints, 90.0, 150.0);

        turretAutoAim = new TurretAutoAim(turret, BLUE_GOAL);

        clawServo.setPosition(0.63);
        leftCompressionServo.setPosition(0.5);
        rightCompressionServo.setPosition(0.5);
        leftHoodServo.setPosition(leftHoodPosition);
        rightHoodPosition = 0.12;
        rightHoodServo.setPosition(rightHoodPosition);
        gateClosed = false;
        gateServo.setPosition(GATE_OPEN);

        telemetry.addData("Status", "Initialized (auto RPM + auto turret)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            if (follower != null) {
                try {
                    follower.update();
                    currentPose = follower.getPose();
                } catch (Exception e) {
                    // keep last pose
                }
            }

            boolean gamepad2TouchpadNow = false;
            try { gamepad2TouchpadNow = gamepad2.touchpad; } catch (Throwable t) {
                gamepad2TouchpadNow = (gamepad2.left_stick_button && gamepad2.right_stick_button);
            }
            if (gamepad2TouchpadNow && !gamepad2TouchpadLast) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveController.stop();
                telemetry.addData("Reset", "Turret encoder set to zero!");
                telemetry.update();
            }
            gamepad2TouchpadLast = gamepad2TouchpadNow;

            double axial   = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;
            double driveSpeed = 1.0;
            driveController.setDrive(axial, lateral, yaw, driveSpeed);

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

            boolean dpadUpNow = gamepad1.dpad_up || gamepad2.dpad_up;
            if (dpadUpNow && !dpadUpLast) {
                gateClosed = !gateClosed;
                gateServo.setPosition(gateClosed ? GATE_CLOSED : GATE_OPEN);
            }
            dpadUpLast = dpadUpNow;

            boolean yNow = gamepad1.y || gamepad2.y;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);

            double targetRpm = flywheelVersatile.getFinalTargetRPM(currentPose);
            flywheel.setTargetRPM(targetRpm);
            flywheel.update(nowMs, yNow);

            if (flywheel.isAtTarget()) {
                final int RUMBLE_MS = 200;
                try { gamepad1.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
            }

            // Auto-aim turret toward blue goal
            turretAutoAim.update(currentPose);

            boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
            if (leftTriggerNow) {
                intakeMotor.setPower(-1.0);
                leftCompressionServo.setPosition(0.0);
                rightCompressionServo.setPosition(1.0);
            } else {
                if ((gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                    intakeMotor.setPower(1.0);
                    leftCompressionServo.setPosition(1.0);
                    rightCompressionServo.setPosition(0.0);
                } else {
                    intakeMotor.setPower(0.0);
                    leftCompressionServo.setPosition(0.5);
                    rightCompressionServo.setPosition(0.5);
                }
            }

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

            telemetry.addData("Pose", currentPose != null
                    ? String.format("(%.1f, %.1f, %.1f°)", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()))
                    : "N/A");
            telemetry.addData("Dist->Goal", currentPose != null
                    ? String.format("%.1f", flywheelVersatile.getLastDistance()) : "N/A");
            telemetry.addData("Base RPM (model)", String.format("%.1f", flywheelVersatile.getLastBaseRpm()));
            telemetry.addData("Trim RPM", String.format("%.1f", flywheelVersatile.getTrimRpm()));
            telemetry.addData("Target RPM", String.format("%.1f", targetRpm));
            telemetry.addData("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
            telemetry.addData("Fly AtTarget", flywheel.isAtTarget());
            telemetry.addData("Gate", gateClosed ? "CLOSED" : "OPEN");
            telemetry.addData("Gate Pos", gateServo.getPosition());
            telemetry.addData("Turret Power", turret.getPower());

            telemetry.update();
        }
    }
}
