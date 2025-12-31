package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretGoalAimer;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

@TeleOp(name="HORS Experimental", group="Linear OpMode")
public class wildexperiment extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;

    // Gate servo
    private Servo gateServo;
    private boolean gateClosed = false;
    private static final double GATE_OPEN = 0.67; // lmao
    private static final double GATE_CLOSED = 0.5;

    // REV Digital LED Indicator (active-low) using hardware map names led1 (red) and led2 (green)
    private DigitalChannel ledLineRed;   // led1
    private DigitalChannel ledLineGreen; // led2

    // Subsystems
    private TurretGoalAimer turretGoalAimer;
    private DriveController driveController;
    private Flywheel flywheel;

    // PedroPathing follower (for pose to aim the turret)
    private Follower follower;
    private Pose currentPose = new Pose(20, 122, Math.toRadians(135)); // updated start pose
    // Single goal pose to track (blue goal); now includes heading 135°
    private static final Pose GOAL_POSE = new Pose(14, 134, Math.toRadians(135));

    // UI / debounce and other small state
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean xPressedLast = false;
    private boolean yPressedLast = false;

    // hood/claw
    private double leftHoodPosition = 0.12;
    private double rightHoodPosition = 0.12;

    private BNO055IMU imu;            // existing expansion-hub IMU (named "imu" in config)
    private BNO055IMU pinpointImu;    // optional pinpoint IMU (named "pinpoint" in config)
    private BNO055IMU turretImu;      // the IMU actually used by the turret (pinpoint if present otherwise imu)

    @Override
    public void runOpMode() {

        // Hardware map
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2"); // new secondary shooter motor
        turret = hardwareMap.get(DcMotor.class, "turret");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        try {
            ledLineRed = hardwareMap.get(DigitalChannel.class, "led1");
            ledLineGreen = hardwareMap.get(DigitalChannel.class, "led2");
            ledLineRed.setMode(DigitalChannel.Mode.OUTPUT);
            ledLineGreen.setMode(DigitalChannel.Mode.OUTPUT);
            // Active-low: true = off, false = on
            ledLineRed.setState(true);
            ledLineGreen.setState(true);
        } catch (Exception e) {
            ledLineRed = null;
            ledLineGreen = null;
        }

        // Directions & modes
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD); // opposite of shooter
        turret.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // mirror shooter mode
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        turretGoalAimer = new TurretGoalAimer(turret, turretImu, telemetry);
        turretGoalAimer.setTargetPose(GOAL_POSE);
        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new Flywheel(shooter, telemetry);

        // PedroPathing follower for pose to aim turret
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));  // match start pose
            follower.update();
            currentPose = follower.getPose();
        } catch (Exception e) {
            follower = null; // fail gracefully
        }

        // Initial positions
        clawServo.setPosition(0.63);
        leftHoodServo.setPosition(leftHoodPosition);
        rightHoodServo.setPosition(rightHoodPosition);
        gateClosed = false;
        gateServo.setPosition(GATE_OPEN);
        updateGateLed(); // reflect initial gate state

        String imuUsed = (turretImu == pinpointImu && pinpointImu != null) ? "pinpoint" :
                (turretImu == imu && imu != null) ? "imu (expansion hub)" : "none";
        telemetry.addData("Status", "Initialized (mode = CLOSE)");
        telemetry.addData("Turret IMU", imuUsed);
        telemetry.update();

        // ensure subsystems are ready
        turretGoalAimer.captureReferences();
        turretGoalAimer.resetPidState();

        waitForStart();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // Update pose for turret aiming
            if (follower != null) {
                try {
                    follower.update();
                    currentPose = follower.getPose();
                } catch (Exception ignored) {}
            }

            // Turret control (manual override on right stick X)
            boolean manualTurret = Math.abs(gamepad2.right_stick_x) > 0.05;
            double manualPower = -gamepad2.right_stick_x; // invert if needed
            turretGoalAimer.update(manualTurret, manualPower, currentPose);

            // Drive
            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x;
            double driveSpeed = 1.0;
            driveController.setDrive(axial, lateral, yaw, driveSpeed);

            // Gate toggle (Y)
            boolean yPressedNow = gamepad1.y || gamepad2.y;
            if (yPressedNow && !yPressedLast) {
                gateClosed = !gateClosed;
                gateServo.setPosition(gateClosed ? GATE_CLOSED : GATE_OPEN);
                updateGateLed();
            }
            yPressedLast = yPressedNow;

            // Basic telemetry
            telemetry.addData("Gate", gateClosed ? "CLOSED" : "OPEN");
            telemetry.addData("Gate Pos", gateServo.getPosition());

            String imuUsedNow = (turretImu == pinpointImu && pinpointImu != null) ? "pinpoint" :
                    (turretImu == imu && imu != null) ? "imu (exp hub)" : "none";
            telemetry.addData("Turret IMU Used", imuUsedNow);
            telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));
            telemetry.addData("Robot Heading(deg)", String.format("%.1f", Math.toDegrees(currentPose.getHeading())));
            telemetry.addData("Turret offset(deg)", String.format("%.2f", turretGoalAimer.getLastOffsetDeg()));
            telemetry.update();
        }
    }

    private void updateGateLed() {
        if (ledLineRed == null || ledLineGreen == null) return;
        // Active-low: closed -> red on, green off. Open -> green on, red off.
        if (gateClosed) {
            ledLineRed.setState(false);
            ledLineGreen.setState(true);
        } else {
            ledLineRed.setState(true);
            ledLineGreen.setState(false);
        }
    }
}