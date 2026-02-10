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

@TeleOp(name = "HORS RED 🔴", group = "Linear OpMode")
public class wildexperimentRed extends LinearOpMode {

    // ========== RED ALLIANCE SETTINGS ==========
    private static final boolean IS_RED_ALLIANCE = true;
    private static final Pose START_POSE = CalibrationPoints.getStartPose(true);

    // Drive + subsystems
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private DcMotor shooter;
    private DcMotor shooter2;
    private DcMotor turret;
    private DcMotor intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo;
    private Servo rightHoodServo;
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

    // IMUs
    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;

    // Pose tracking
    private Follower follower;
    private Pose currentPose = new Pose();

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // ==================== HARDWARE MAP ====================
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
        } catch (Exception ignored) {
        }

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
            } catch (Exception ignored) {
            }
        }

        // ==================== FOLLOWER INIT ====================
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(START_POSE);
        } catch (Exception e) {
            telemetry.addData("Error", "Follower initialization failed!");
            follower = null;
        }

        String imuUsed = (pinpoint != null) ? "pinpoint" : (imu != null) ? "imu (expansion hub)" : "none";

        // ==================== CREATE CONTROLLERS ====================
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
                CalibrationPoints.HOOD_MIN, CalibrationPoints.RIGHT_HOOD_CLOSE,
                CalibrationPoints.HOOD_MIN, CalibrationPoints.HOOD_MAX,
                CalibrationPoints.HOOD_LEFT_STEP, CalibrationPoints.HOOD_RIGHT_STEP,
                CalibrationPoints.HOOD_DEBOUNCE_MS
        );

        // ==================== FLYWHEEL VERSATILE (AUTO MIRRORS FOR RED) ====================
        flywheelVersatile = new FlywheelVersatile(
                flywheel,
                CalibrationPoints.BLUE_GOAL,
                CalibrationPoints.FLYWHEEL_CALIBRATION_DATA,
                CalibrationPoints.FLYWHEEL_MIN_RPM,
                CalibrationPoints.FLYWHEEL_MAX_RPM
        );
        flywheelVersatile.setRedAlliance(IS_RED_ALLIANCE);

        // ==================== HOOD VERSATILE (AUTO MIRRORS FOR RED) ====================
        hoodVersatile = new HoodVersatile(
                hoodController,
                CalibrationPoints.BLUE_GOAL,
                CalibrationPoints.HOOD_CLOSE_POSE,
                CalibrationPoints.HOOD_FAR_POSE,
                CalibrationPoints.HOOD_MIN,
                CalibrationPoints.HOOD_MAX
        );
        hoodVersatile.setRedAlliance(IS_RED_ALLIANCE);

        // ==================== INITIAL STATE ====================
        gateController.setGateClosed(true);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "RED 🔴");
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

        // ==================== MAIN LOOP ====================
        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            // Refresh pinpoint
            if (pinpoint != null) {
                try {
                    pinpoint.update();
                } catch (Exception ignored) {
                }
            }

            // Update follower and pose
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

            // ========== A BUTTON (GP1) = RESET POSE ==========
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

            // ========== TOUCHPAD (GP1) OR RB (GP2) = TOGGLE AUTO MODES ==========
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

            // ========== DRIVE ==========
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 1.0);

            // ========== FLYWHEEL TOGGLES ==========
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                flywheel.toggleShooterOn();
            }
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

            // ========== GATE TOGGLE (B) ==========
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) {
                gateController.toggleGate();
            }
            bPressedLast = bNow;

            // ========== INTAKE SEQUENCE (Y) ==========
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

            // ========== AUTO FLYWHEEL RPM ==========
            if (autoFlywheelEnabled && follower != null && currentPose != null) {
                double targetRpm = flywheelVersatile.getFinalTargetRPM(currentPose);
                flywheel.setTargetRPM(targetRpm);
            }

            // ========== FLYWHEEL UPDATE ==========
            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            // ========== AUTO HOOD ==========
            if (autoHoodEnabled && follower != null && currentPose != null) {
                hoodVersatile.update(currentPose);
            }

            // ========== HOOD TRIM ==========
            if (autoHoodEnabled) {
                if (gamepad2.left_stick_y < -0.5) {
                    hoodVersatile.adjustTrim(CalibrationPoints.HOOD_TRIM_STEP);
                } else if (gamepad2.left_stick_y > 0.5) {
                    hoodVersatile.adjustTrim(-CalibrationPoints.HOOD_TRIM_STEP);
                }
            }

            // ========== RUMBLE AT TARGET ==========
            if (flywheel.isAtTarget()) {
                try {
                    gamepad1.rumble(200);
                } catch (Throwable ignored) {
                }
                try {
                    gamepad2.rumble(200);
                } catch (Throwable ignored) {
                }
            }

            // ========== TURRET CONTROL ==========
            turretController.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);

            boolean manualNow = false;
            double manualPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) {
                manualNow = true;
                manualPower = 0.35;
            } else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) {
                manualNow = true;
                manualPower = -0.35;
            }
            turretController.update(manualNow, manualPower);

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

            // ========== CLAW (X) ==========
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawController.trigger(nowMs);
            }
            xPressedLast = xNow;
            clawController.update(nowMs);

            // ========== TELEMETRY ==========
            telemetry.addData("Alliance", "RED 🔴");
            telemetry.addData("Flywheel", "%.0f / %.0f rpm %s",
                    flywheel.getCurrentRPM(), flywheel.getTargetRPM(),
                    autoFlywheelEnabled ? "(AUTO)" : "(MANUAL)");

            if (autoFlywheelEnabled) {
                telemetry.addData("Fly Trim", "%.0f rpm", flywheelVersatile.getTrimRpm());
            }

            if (autoHoodEnabled) {
                telemetry.addData("Hood (AUTO)", "%.3f | Dist: %.1f | Trim: %.3f",
                        hoodVersatile.getLastTargetPos(),
                        hoodVersatile.getLastDistance(),
                        hoodVersatile.getTrimPos());
            } else {
                telemetry.addData("Hood (MANUAL)", "%.3f", hoodController.getRightPos());
            }

            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));

            // ========== PANELS TELEMETRY ==========
            panelsTelemetry.debug("Alliance", "RED");
            panelsTelemetry.debug("X", String.format("%.1f", currentPose.getX()));
            panelsTelemetry.debug("Y", String.format("%.1f", currentPose.getY()));
            panelsTelemetry.debug("Heading", String.format("%.1f", Math.toDegrees(currentPose.getHeading())));
            panelsTelemetry.debug("Fly RPM", String.format("%.0f", flywheel.getCurrentRPM()));
            panelsTelemetry.debug("Fly Target", String.format("%.0f", flywheel.getTargetRPM()));
            panelsTelemetry.debug("Fly Trim", String.format("%.0f", flywheelVersatile.getTrimRpm()));
            panelsTelemetry.debug("Hood", String.format("%.3f", hoodVersatile.getLastTargetPos()));
            panelsTelemetry.debug("Hood Trim", String.format("%.3f", hoodVersatile.getTrimPos()));
            panelsTelemetry.debug("Distance", String.format("%.1f", hoodVersatile.getLastDistance()));
            panelsTelemetry.debug("Auto Mode", autoFlywheelEnabled ? "ON" : "OFF");

            telemetry.update();
            panelsTelemetry.update(telemetry);
        }

        turretController.disable();
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

    private void reZeroHeadingAndTurret(BNO055IMU.Parameters imuParams) {
        if (pinpoint != null) {
            try {
                pinpoint.update();
            } catch (Exception ignored) {
            }
        }
        turretController.captureReferences();
        turretController.resetPidState();
    }

    private void resetTurretEncoderAndReferences(BNO055IMU.Parameters imuParams) {
        if (pinpoint != null) {
            try {
                pinpoint.update();
            } catch (Exception ignored) {
            }
        }
        turretController.recenterAndResume(true);
    }
}