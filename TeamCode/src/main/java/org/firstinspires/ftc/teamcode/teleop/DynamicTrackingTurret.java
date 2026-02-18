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
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.tracking.TurretGoalAimer;

@TeleOp(name="Point Tracking HORS 🎯", group="Linear OpMode")
public class DynamicTrackingTurret extends LinearOpMode {

    // Drive + subsystems
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;
    private DigitalChannel turretLimitSwitch;

    private TurretGoalAimer turretAimer;
    private DriveController driveController;
    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;

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

    private boolean isFarMode = false;
    private boolean lastPidfMode = false;

    // Hood presets
    private static final double RIGHT_HOOD_CLOSE = 0.16;
    private static final double RIGHT_HOOD_FAR = 0.24;

    // Gate/Intake constants
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.467;
    private static final long INTAKE_DURATION_MS = 1050;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 400;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;

    // Hood constants
    private static final double HOOD_MIN = 0.12;
    private static final double HOOD_MAX = 0.45;
    private static final double HOOD_LEFT_STEP = 0.025;
    private static final double HOOD_RIGHT_STEP = 0.01;
    private static final long HOOD_DEBOUNCE_MS = 120L;

    // Target tracking
    private static final Pose BLUE_GOAL = new Pose(14, 134, 0);
    private static final Pose RED_GOAL = new Pose(132, 134, 0);
    private static final Pose START_POSE = new Pose(20, 122, Math.toRadians(130));

    private boolean isRedAlliance = false;  // Set to true for red side

    // IMUs
    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;

    // Pose tracking
    private Follower follower;
    private Pose currentPose = new Pose();

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // ==================== HARDWARE INIT ====================
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

        // Turret limit switch
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

        // ==================== MOTOR DIRECTIONS & MODES ====================
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

        // ==================== IMU INIT ====================
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
        } catch (Exception e) {
            imu = null;
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            if (pinpoint != null) pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            pinpoint = null;
        }

        if (imu != null) {
            try {
                imu.initialize(imuParams);
            } catch (Exception ignored) {}
        }

        // ==================== FOLLOWER INIT ====================
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(START_POSE);
        } catch (Exception e) {
            telemetry.addData("Error", "Follower init failed!");
            follower = null;
        }

        // ==================== CONTROLLERS INIT ====================
        turretAimer = new TurretGoalAimer(turret, imu, pinpoint, null);  // No telemetry spam
        turretAimer.setTargetPose(isRedAlliance ? RED_GOAL : BLUE_GOAL);

        if (turretLimitSwitch != null) {
            turretAimer.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new FlywheelController(shooter, shooter2, telemetry, batterySensor);
        flywheel.setShooterOn(false);

        gateController = new GateController(
                gateServo, intakeMotor,
                led1Red, led1Green, led2Red, led2Green,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQUENCE_POWER
        );

        clawController = new ClawController(clawServo, ClawController.CLAW_OPEN, ClawController.CLAW_CLOSED, ClawController.CLAW_CLOSE_MS);
        hoodController = new HoodController(
                leftHoodServo, rightHoodServo,
                HOOD_MIN, RIGHT_HOOD_CLOSE,
                HOOD_MIN, HOOD_MAX,
                HOOD_LEFT_STEP, HOOD_RIGHT_STEP,
                HOOD_DEBOUNCE_MS
        );

        // ==================== INITIAL STATE ====================
        gateController.setGateClosed(true);

        telemetry.addData("Status", "Ready 🎯");
        telemetry.addData("Alliance", isRedAlliance ? "RED 🔴" : "BLUE 🔵");
        telemetry.update();

        turretAimer.captureReferences();
        turretAimer.resetPidState();

        waitForStart();

        if (isStopRequested()) return;

        // Force unfreeze and reset at start
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        reZeroTurret();
        flywheel.setShooterOn(true);

        // ==================== MAIN LOOP ====================
        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            // ========== UPDATE SENSORS ==========
            if (pinpoint != null) {
                try {
                    pinpoint.update();
                } catch (Exception ignored) {}
            }

            if (follower != null) {
                try {
                    follower.update();
                    currentPose = follower.getPose();
                } catch (Exception ignored) {}
            }

            // Fallback pose if null
            if (currentPose == null) {
                currentPose = START_POSE;
            }

            // ========== BUTTON A: FULL RESET ==========
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

            // ========== GP2 TOUCHPAD: TURRET RECALIBRATE ==========
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gamepad2TouchpadLast) {
                reZeroTurret();
                gamepad2.rumble(200);
            }
            gamepad2TouchpadLast = gp2Touch;

            // ========== FAR/CLOSE MODE TOGGLE ==========
            boolean touchpadNow = getTouchpad(gamepad1) || gamepad2.right_bumper;
            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
            }
            touchpadPressedLast = touchpadNow;

            // ========== DRIVE ==========
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 1.0);

            // ========== FLYWHEEL TOGGLES ==========
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) flywheel.adjustTargetRPM(-50.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) flywheel.adjustTargetRPM(50.0);
            dpadRightLast = dpadRightNow;

            // ========== GATE TOGGLE ==========
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) {
                gateController.toggleGate();
            }
            bPressedLast = bNow;

            // ========== INTAKE SEQUENCE ==========
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy()) {
                gateController.startIntakeSequence(nowMs);
            }
            yPressedLast = yNow;

            // ========== GATE UPDATE ==========
            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) {
                clawController.trigger(nowMs);
            }

            // ========== FLYWHEEL UPDATE ==========
            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            boolean currentPidfMode = flywheel.isUsingFarCoefficients();
            if (currentPidfMode != lastPidfMode) {
                hoodController.setRightPosition(currentPidfMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                lastPidfMode = currentPidfMode;
            }

            // ========== RUMBLE AT TARGET ==========
            if (flywheel.isAtTarget()) {
                try { gamepad1.rumble(200); } catch (Throwable ignored) {}
                try { gamepad2.rumble(200); } catch (Throwable ignored) {}
            }

            // ========== HOMING SWEEP ==========
            turretAimer.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);

            // ========== TURRET CONTROL ==========
            boolean manualNow = false;
            double manualPower = 0.0;

            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.5) {
                manualNow = true;
                manualPower = 0.35;
            } else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.5) {
                manualNow = true;
                manualPower = -0.35;
            }

            turretAimer.update(manualNow, manualPower, currentPose);

            // ========== INTAKE MANUAL ==========
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

            // ========== CLAW ==========
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawController.trigger(nowMs);
            }
            xPressedLast = xNow;
            clawController.update(nowMs);

            // ========== HOOD ADJUSTMENTS ==========
            if (gamepad1.b) hoodController.nudgeLeftDown(nowMs);

            // ========== ESSENTIAL TELEMETRY ONLY ==========
            telemetry.addData("RPM", "%.0f → %.0f",
                    flywheel.getCurrentRPM(), flywheel.getTargetRPM());
            telemetry.addData("Turret", "%s | %d err",
                    turretAimer.isFreezeMode() ? "HOLD" : (turretAimer.isHomingMode() ? "HOME" : "TRACK"),
                    turretAimer.getLastErrorTicks());
            telemetry.addData("Pose", "(%.0f, %.0f, %.0f°)",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

            telemetry.update();
        }

        turretAimer.update(true, 0.0, null);
    }

    private boolean getTouchpad(com.qualcomm.robotcore.hardware.Gamepad gp) {
        try {
            return gp.touchpad;
        } catch (Throwable t) {
            return gp.left_stick_button && gp.right_stick_button;
        }
    }

    private LED getLedSafe(String name) {
        try {
            return hardwareMap.get(LED.class, name);
        } catch (Exception ignored) {
            return null;
        }
    }

    private void reZeroTurret() {
        if (pinpoint != null) {
            try {
                pinpoint.update();
            } catch (Exception ignored) {}
        }
        turretAimer.captureReferences();
        turretAimer.resetPidState();
    }

    private void resetTurretEncoderAndReferences(BNO055IMU.Parameters imuParams) {
        if (pinpoint != null) {
            try {
                pinpoint.update();
            } catch (Exception ignored) {}
        }
        turretAimer.recenterAndResume(true);
    }
}