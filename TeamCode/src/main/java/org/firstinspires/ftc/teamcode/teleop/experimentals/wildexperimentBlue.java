package org.firstinspires.ftc.teamcode.teleop.experimentals;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelVersatile;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.subsystems.HoodVersatile;
import org.firstinspires.ftc.teamcode.tracking.CalibrationPoints;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

@TeleOp(name = "HORS BLUE 🔵", group = "Linear OpMode")
public class wildexperimentBlue extends LinearOpMode {

    private static final boolean IS_RED_ALLIANCE = false;
    private static final Pose START_POSE = CalibrationPoints.BLUE_START_POSE;

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo, leftHoodServo, rightHoodServo, gateServo;
    private DigitalChannel turretLimitSwitch;

    private TurretController turretController;
    private DriveController driveController;
    private FlywheelController flywheel;
    private FlywheelVersatile flywheelVersatile;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;
    private HoodVersatile hoodVersatile;

    private TelemetryManager panelsTelemetry;

    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean xPressedLast = false;
    private boolean bPressedLast = false;
    private boolean yPressedLast = false;
    private boolean touchpadPressedLast = false;
    private boolean gamepad2TouchpadLast = false;
    private boolean aPressedLast = false;

    private boolean isFarMode = false;
    private boolean autoHoodEnabled = true;
    private boolean autoFlywheelEnabled = true;

    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;

    private Follower follower;
    private Pose currentPose;

    @Override
    public void runOpMode() {

        currentPose = START_POSE;

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Hardware init
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        turret = hardwareMap.get(DcMotor.class, "turret");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        try {
            turretLimitSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
            turretLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            turretLimitSwitch = null;
        }

        LED led1Red = getLedSafe("led_1_red");
        LED led1Green = getLedSafe("led_1_green");
        LED led2Red = getLedSafe("led_2_red");
        LED led2Green = getLedSafe("led_2_green");

        VoltageSensor batterySensor = null;
        try {
            batterySensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {}

        // Motor setup
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        turret.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU setup
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        try { imu = hardwareMap.get(BNO055IMU.class, "imu"); } catch (Exception e) { imu = null; }
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            if (pinpoint != null) pinpoint.resetPosAndIMU();
        } catch (Exception e) { pinpoint = null; }

        if (imu != null) {
            try { imu.initialize(imuParams); } catch (Exception ignored) {}
        }

        // Follower
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(START_POSE);
        } catch (Exception e) {
            telemetry.addData("Error", "Follower init failed!");
            follower = null;
        }

        // Controllers
        turretController = new TurretController(turret, imu, pinpoint, telemetry);
        if (turretLimitSwitch != null) {
            turretController.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        flywheel = new FlywheelController(shooter, shooter2, telemetry, batterySensor);
        flywheel.setShooterOn(false);

        gateController = new GateController(
                gateServo, intakeMotor,
                led1Red, led1Green, led2Red, led2Green,
                CalibrationPoints.GATE_OPEN, CalibrationPoints.GATE_CLOSED,
                CalibrationPoints.INTAKE_DURATION_MS, CalibrationPoints.CLAW_TRIGGER_BEFORE_END_MS,
                CalibrationPoints.INTAKE_SEQUENCE_POWER
        );

        clawController = new ClawController(
                clawServo,
                CalibrationPoints.CLAW_OPEN,
                CalibrationPoints.CLAW_CLOSED,
                CalibrationPoints.CLAW_CLOSE_MS
        );

        hoodController = new HoodController(
                leftHoodServo, rightHoodServo,
                CalibrationPoints.HOOD_MIN, CalibrationPoints.HOOD_MIN,
                CalibrationPoints.HOOD_MIN, CalibrationPoints.HOOD_MAX,
                CalibrationPoints.HOOD_LEFT_STEP, CalibrationPoints.HOOD_RIGHT_STEP,
                CalibrationPoints.HOOD_DEBOUNCE_MS
        );

        // Versatile controllers
        flywheelVersatile = new FlywheelVersatile(
                flywheel,
                CalibrationPoints.BLUE_GOAL,
                CalibrationPoints.FLYWHEEL_CALIBRATION_DATA,
                CalibrationPoints.FLYWHEEL_MIN_RPM,
                CalibrationPoints.FLYWHEEL_MAX_RPM
        );
        flywheelVersatile.setRedAlliance(IS_RED_ALLIANCE);

        hoodVersatile = new HoodVersatile(
                hoodController,
                CalibrationPoints.BLUE_GOAL,
                CalibrationPoints.HOOD_CLOSE_POSE,
                CalibrationPoints.HOOD_FAR_POSE,
                CalibrationPoints.HOOD_MIN,
                CalibrationPoints.HOOD_MAX
        );
        hoodVersatile.setRedAlliance(IS_RED_ALLIANCE);

        // Initial values
        double initialRpm = flywheelVersatile.getFinalTargetRPM(START_POSE);
        double initialHood = hoodVersatile.getFinalTargetPosition(START_POSE);

        flywheel.setTargetRPM(initialRpm);
        hoodController.setRightPosition(initialHood);
        gateController.setGateClosed(true);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "BLUE 🔵");
        telemetry.addData("Start", "(%.1f, %.1f)", START_POSE.getX(), START_POSE.getY());
        telemetry.addData("Start Dist", "%.1f", CalibrationPoints.START_DISTANCE);
        telemetry.addData("Initial RPM", "%.0f", initialRpm);
        telemetry.addData("Initial Hood", "%.3f", initialHood);
        telemetry.update();

        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();

        if (isStopRequested()) {
            turretController.disable();
            return;
        }

        reZeroHeadingAndTurret(imuParams);
        flywheel.setShooterOn(true);

        // Main loop
        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            if (pinpoint != null) {
                try { pinpoint.update(); } catch (Exception ignored) {}
            }

            // Get pose from follower - DIRECTLY use it
            if (follower != null) {
                follower.update();
                Pose newPose = follower.getPose();
                if (newPose != null) {
                    currentPose = newPose;
                }
            }

            // Reset controls
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gamepad2TouchpadLast) {
                if (follower != null) follower.setStartingPose(START_POSE);
                currentPose = START_POSE;
                resetTurretEncoderAndReferences(imuParams);
                driveController.stop();
                gamepad2.rumble(300);
            }
            gamepad2TouchpadLast = gp2Touch;

            boolean aNow = gamepad1.a;
            if (aNow && !aPressedLast) {
                if (follower != null) follower.setStartingPose(START_POSE);
                currentPose = START_POSE;
                resetTurretEncoderAndReferences(imuParams);
                driveController.stop();
                gamepad1.rumble(300);
            }
            aPressedLast = aNow;

            // Toggle auto modes
            boolean touchpadNow = getTouchpad(gamepad1) || gamepad2.right_bumper;
            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                autoHoodEnabled = !autoHoodEnabled;
                autoFlywheelEnabled = !autoFlywheelEnabled;
                if (!autoHoodEnabled) {
                    hoodController.setRightPosition(isFarMode ?
                            CalibrationPoints.RIGHT_HOOD_FAR : CalibrationPoints.RIGHT_HOOD_CLOSE);
                }
            }
            touchpadPressedLast = touchpadNow;

            // Drive
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 1.0);

            // Flywheel controls
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) {
                if (autoFlywheelEnabled) flywheelVersatile.adjustTrim(-50.0);
                else flywheel.adjustTargetRPM(-50.0);
            }
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) {
                if (autoFlywheelEnabled) flywheelVersatile.adjustTrim(50.0);
                else flywheel.adjustTargetRPM(50.0);
            }
            dpadRightLast = dpadRightNow;

            // Gate
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) gateController.toggleGate();
            bPressedLast = bNow;

            // Intake sequence
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy()) gateController.startIntakeSequence(nowMs);
            yPressedLast = yNow;

            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) clawController.trigger(nowMs);

            // AUTO FLYWHEEL
            if (autoFlywheelEnabled) {
                double targetRpm = flywheelVersatile.getFinalTargetRPM(currentPose);
                flywheel.setTargetRPM(targetRpm);
            }

            // Flywheel update
            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            // AUTO HOOD
            if (autoHoodEnabled) {
                hoodVersatile.update(currentPose);
            }

            // Hood trim
            if (autoHoodEnabled) {
                if (gamepad2.left_stick_y < -0.5) hoodVersatile.adjustTrim(CalibrationPoints.HOOD_TRIM_STEP);
                else if (gamepad2.left_stick_y > 0.5) hoodVersatile.adjustTrim(-CalibrationPoints.HOOD_TRIM_STEP);
            }

            // Rumble
            if (flywheel.isAtTarget()) {
                try { gamepad1.rumble(200); } catch (Throwable ignored) {}
                try { gamepad2.rumble(200); } catch (Throwable ignored) {}
            }

            // Turret
            turretController.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);
            boolean manualNow = false;
            double manualPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) { manualNow = true; manualPower = 0.35; }
            else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) { manualNow = true; manualPower = -0.35; }
            turretController.update(manualNow, manualPower);

            // Intake manual
            if (!gateController.isBusy()) {
                boolean leftTriggerNow = gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1;
                if (leftTriggerNow) intakeMotor.setPower(-1.0);
                else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) intakeMotor.setPower(1.0);
                else intakeMotor.setPower(0.0);
            }

            // Claw
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) clawController.trigger(nowMs);
            xPressedLast = xNow;
            clawController.update(nowMs);

            // ========== TELEMETRY ==========
            telemetry.addData("Alliance", "BLUE 🔵");
            telemetry.addData("Auto", "Fly:%s Hood:%s",
                    autoFlywheelEnabled ? "ON" : "OFF",
                    autoHoodEnabled ? "ON" : "OFF");

            telemetry.addData("Flywheel", "%.0f / %.0f rpm",
                    flywheel.getCurrentRPM(), flywheel.getTargetRPM());
            telemetry.addData("Fly Trim", "%.0f rpm", flywheelVersatile.getTrimRpm());

            telemetry.addData("Hood", "%.3f | Dist: %.1f",
                    hoodVersatile.getLastTargetPos(),
                    hoodVersatile.getLastDistance());
            telemetry.addData("Hood Trim", "%.3f", hoodVersatile.getTrimPos());

            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

            // DEBUG INFO
            telemetry.addLine("--- DEBUG ---");
            telemetry.addData("Fly Updates", flywheelVersatile.getUpdateCount());
            telemetry.addData("Fly Reject", flywheelVersatile.getLastRejectReason());
            telemetry.addData("Hood Updates", hoodVersatile.getUpdateCount());
            telemetry.addData("Hood Reject", hoodVersatile.getLastRejectReason());

            // Panels
            panelsTelemetry.debug("Alliance", "BLUE");
            panelsTelemetry.debug("X", String.format("%.1f", currentPose.getX()));
            panelsTelemetry.debug("Y", String.format("%.1f", currentPose.getY()));
            panelsTelemetry.debug("Fly RPM", String.format("%.0f", flywheel.getCurrentRPM()));
            panelsTelemetry.debug("Fly Target", String.format("%.0f", flywheel.getTargetRPM()));
            panelsTelemetry.debug("Hood", String.format("%.3f", hoodVersatile.getLastTargetPos()));
            panelsTelemetry.debug("Distance", String.format("%.1f", hoodVersatile.getLastDistance()));

            telemetry.update();
            panelsTelemetry.update(telemetry);
        }

        turretController.disable();
    }

    private boolean getTouchpad(com.qualcomm.robotcore.hardware.Gamepad gp) {
        try { return gp.touchpad; }
        catch (Throwable t) { return gp.left_stick_button && gp.right_stick_button; }
    }

    private LED getLedSafe(String name) {
        try { return hardwareMap.get(LED.class, name); }
        catch (Exception ignored) { return null; }
    }

    private void reZeroHeadingAndTurret(BNO055IMU.Parameters imuParams) {
        if (pinpoint != null) { try { pinpoint.update(); } catch (Exception ignored) {} }
        turretController.captureReferences();
        turretController.resetPidState();
    }

    private void resetTurretEncoderAndReferences(BNO055IMU.Parameters imuParams) {
        if (pinpoint != null) { try { pinpoint.update(); } catch (Exception ignored) {} }
        turretController.recenterAndResume(true);
    }
}