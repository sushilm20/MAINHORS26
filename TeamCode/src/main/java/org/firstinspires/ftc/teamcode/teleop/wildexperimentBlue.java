package org.firstinspires.ftc.teamcode.teleop;

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
import org.firstinspires.ftc.teamcode.subsystems.FlywheelVersatile.CalibrationPoint;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.subsystems.HoodVersatile;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="HORS experiment BLUE 🔵", group="Linear OpMode")
public class wildexperimentBlue extends LinearOpMode {

    // Drive + subsystems
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;
    private DigitalChannel turretLimitSwitch;

    private TurretController turretController;
    private DriveController driveController;
    private FlywheelController flywheel;
    private FlywheelVersatile flywheelVersatile;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;
    private HoodVersatile hoodVersatile;

    // Telemetry
    private TelemetryManager panelsTelemetry;

    // Toggles / state
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean xPressedLast = false;
    private boolean bPressedLast = false;
    private boolean yPressedLast = false;
    private boolean touchpadPressedLast = false;
    private boolean gamepad2TouchpadLast = false;
    private boolean aPressedLast = false;
    private boolean dpadUpLast = false;

    private boolean isFarMode = false;
    private boolean autoHoodEnabled = true;
    private boolean autoFlywheelEnabled = true;

    // ========== BLUE ALLIANCE SETTINGS ==========
    private static final boolean IS_RED_ALLIANCE = false;

    // BLUE start pose
    private static final Pose START_POSE = new Pose(20, 122, Math.toRadians(130));

    // BLUE goal pose
    private static final Pose BLUE_GOAL = new Pose(0, 144, 0);

    // Hood calibration poses (BLUE)
    private static final Pose HOOD_CLOSE_POSE = new Pose(20, 122, 0);
    private static final Pose HOOD_FAR_POSE = new Pose(72, 12, 0);

    // Hood presets (for manual mode)
    private static final double RIGHT_HOOD_CLOSE = 0.16;
    private static final double RIGHT_HOOD_FAR = 0.24;

    // Gate/Intake constants
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.467;
    private static final long INTAKE_DURATION_MS = 1050;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 400;
    private static final double INTKE_SEQUENCE_POWER = 1.0;

    // Claw constants
    private static final double CLAW_OPEN = 0.63;
    private static final double CLAW_CLOSED = 0.2;
    private static final long CLAW_CLOSE_MS = 500L;

    // Hood constants
    private static final double HOOD_MIN = 0.12;
    private static final double HOOD_MAX = 0.45;
    private static final double HOOD_LEFT_STEP = 0.025;
    private static final double HOOD_RIGHT_STEP = 0.01;
    private static final long HOOD_DEBOUNCE_MS = 120L;
    private static final double HOOD_TRIM_STEP = 0.005;

    // IMUs
    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;

    // Pose tracking
    private Follower follower;
    private Pose currentPose = new Pose();

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Hardware map
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

        // Optional: turret limit switch
        try {
            turretLimitSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
            turretLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            turretLimitSwitch = null;
        }

        // LEDs
        LED led1Red = getLedSafe("led_1_red");
        LED led1Green = getLedSafe("led_1_green");
        LED led2Red = getLedSafe("led_2_red");
        LED led2Green = getLedSafe("led_2_green");

        // Voltage sensor
        VoltageSensor batterySensor = null;
        try {
            batterySensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {}

        // Directions & modes
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

        // IMU init
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

        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(START_POSE);
        } catch (Exception e) {
            telemetry.addData("Error", "Follower initialization failed!");
            follower = null;
        }

        String imuUsed = (pinpoint != null) ? "pinpoint" : (imu != null) ? "imu (expansion hub)" : "none";

        // Create controllers
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
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTKE_SEQUENCE_POWER
        );

        clawController = new ClawController(clawServo, CLAW_OPEN, CLAW_CLOSED, CLAW_CLOSE_MS);
        hoodController = new HoodController(
                leftHoodServo, rightHoodServo,
                HOOD_MIN, RIGHT_HOOD_CLOSE,
                HOOD_MIN, HOOD_MAX,
                HOOD_LEFT_STEP, HOOD_RIGHT_STEP,
                HOOD_DEBOUNCE_MS
        );

        // ========== FLYWHEEL VERSATILE (BLUE calibration) ==========
        List<CalibrationPoint> calibrationPoints = Arrays.asList(
                new CalibrationPoint(new Pose(48, 96, 135), 2300),
                new CalibrationPoint(new Pose(60, 125, 135), 2400),
                new CalibrationPoint(new Pose(60, 82, 135), 2500),
                new CalibrationPoint(new Pose(72, 72, 135), 2650),
                new CalibrationPoint(new Pose(72, 120, 167), 2550),
                new CalibrationPoint(new Pose(95, 120, 135), 2850),
                new CalibrationPoint(new Pose(52, 14, 135), 3750)
        );
        flywheelVersatile = new FlywheelVersatile(flywheel, BLUE_GOAL, calibrationPoints, 2300, 4000);
        flywheelVersatile.setRedAlliance(IS_RED_ALLIANCE);  // false for blue

        // ========== HOOD VERSATILE (BLUE calibration) ==========
        hoodVersatile = new HoodVersatile(
                hoodController,
                BLUE_GOAL,
                HOOD_CLOSE_POSE,
                HOOD_FAR_POSE,
                HOOD_MIN,
                HOOD_MAX
        );
        hoodVersatile.setRedAlliance(IS_RED_ALLIANCE);  // false for blue

        // Initial positions
        gateController.setGateClosed(true);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "BLUE 🔵");
        telemetry.addData("Start Pose", "(%.1f, %.1f, %.1f°)",
                START_POSE.getX(), START_POSE.getY(), Math.toDegrees(START_POSE.getHeading()));
        telemetry.addData("Turret IMU", imuUsed);
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

        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            // Refresh pinpoint
            if (pinpoint != null) {
                try { pinpoint.update(); } catch (Exception ignored) {}
            }

            if (follower != null) {
                follower.update();
                currentPose = follower.getPose();
            }

            // ========== TOUCHPAD (GP2) = RESET POSE TO START ==========
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gamepad2TouchpadLast) {
                if (follower != null) {
                    follower.setStartingPose(START_POSE);
                }
                resetTurretEncoderAndReferences(imuParams);
                driveController.stop();
                gamepad2.rumble(300);
            }
            gamepad2TouchpadLast = gp2Touch;

            // A button (GP1) = reset pose
            boolean aNow = gamepad1.a;
            if (aNow && !aPressedLast) {
                if (follower != null) {
                    follower.setStartingPose(START_POSE);
                }
                resetTurretEncoderAndReferences(imuParams);
                driveController.stop();
                gamepad1.rumble(300);
            }
            aPressedLast = aNow;

            // Touchpad (GP1) or RB (GP2) = toggle auto modes
            boolean touchpadNow = getTouchpad(gamepad1) || gamepad2.right_bumper;
            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                autoHoodEnabled = !autoHoodEnabled;
                autoFlywheelEnabled = !autoFlywheelEnabled;
                if (!autoHoodEnabled) {
                    hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                }
            }
            touchpadPressedLast = touchpadNow;

            // Drive
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 1.0);

            // Flywheel toggles
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) {
                if (autoFlywheelEnabled) {
                    flywheelVersatile.adjustTrim(-50.0);
                } else {
                    flywheel.adjustTargetRPM(-50.0);
                }
            }
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) {
                if (autoFlywheelEnabled) {
                    flywheelVersatile.adjustTrim(50.0);
                } else {
                    flywheel.adjustTargetRPM(50.0);
                }
            }
            dpadRightLast = dpadRightNow;

            // Gate toggle (B)
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) {
                gateController.toggleGate();
            }
            bPressedLast = bNow;

            // Intake sequence (Y)
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy()) {
                gateController.startIntakeSequence(nowMs);
            }
            yPressedLast = yNow;

            // Gate update
            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) {
                clawController.trigger(nowMs);
            }

            // Auto flywheel RPM
            if (autoFlywheelEnabled && follower != null && currentPose != null) {
                double targetRpm = flywheelVersatile.getFinalTargetRPM(currentPose);
                flywheel.setTargetRPM(targetRpm);
            }

            // Flywheel update
            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            // Auto hood
            if (autoHoodEnabled && follower != null && currentPose != null) {
                hoodVersatile.update(currentPose);
            }

            // Hood trim
            if (autoHoodEnabled) {
                if (gamepad2.left_stick_y < -0.5) {
                    hoodVersatile.adjustTrim(HOOD_TRIM_STEP);
                } else if (gamepad2.left_stick_y > 0.5) {
                    hoodVersatile.adjustTrim(-HOOD_TRIM_STEP);
                }
            }

            // Rumble at target
            if (flywheel.isAtTarget()) {
                try { gamepad1.rumble(200); } catch (Throwable ignored) {}
                try { gamepad2.rumble(200); } catch (Throwable ignored) {}
            }

            // Turret control
            turretController.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);

            boolean manualNow = false;
            double manualPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) {
                manualNow = true; manualPower = 0.35;
            } else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) {
                manualNow = true; manualPower = -0.35;
            }
            turretController.update(manualNow, manualPower);

            // Intake manual
            if (!gateController.isBusy()) {
                boolean leftTriggerNow = gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1;
                if (leftTriggerNow) {
                    intakeMotor.setPower(-1.0);
                } else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                    intakeMotor.setPower(1.0);
                } else {
                    intakeMotor.setPower(0.0);
                }
            }

            // Claw (X)
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawController.trigger(nowMs);
            }
            xPressedLast = xNow;
            clawController.update(nowMs);

            // Telemetry
            telemetry.addData("Alliance", "BLUE 🔵");
            telemetry.addData("Flywheel", "%.0f / %.0f rpm %s",
                    flywheel.getCurrentRPM(), flywheel.getTargetRPM(),
                    autoFlywheelEnabled ? "(AUTO)" : "(MANUAL)");

            if (autoHoodEnabled) {
                telemetry.addData("Hood (AUTO)", "%.3f | Dist: %.1f",
                        hoodVersatile.getLastTargetPos(), hoodVersatile.getLastDistance());
            } else {
                telemetry.addData("Hood (MANUAL)", "%.3f", hoodController.getRightPos());
            }

            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

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
        if (pinpoint != null) {
            try { pinpoint.update(); } catch (Exception ignored) {}
        }
        turretController.captureReferences();
        turretController.resetPidState();
    }

    private void resetTurretEncoderAndReferences(BNO055IMU.Parameters imuParams) {
        if (pinpoint != null) {
            try { pinpoint.update(); } catch (Exception ignored) {}
        }
        turretController.recenterAndResume(true);
    }
}