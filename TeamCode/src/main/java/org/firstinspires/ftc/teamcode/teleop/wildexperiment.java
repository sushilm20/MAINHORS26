package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

@TeleOp(name="HORS EXPERIMENTAL 🤖", group="Linear OpMode")
public class wildexperiment extends LinearOpMode {

    // Drive + subsystems
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;

    private TurretController turretController;
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
    private boolean aPressedLast = false; // gamepad1 A reset latch

    private boolean isFarMode = false;
    private boolean lastPidfMode = false; // Track PIDF mode changes for telemetry

    // Hood presets
    private static final double RIGHT_HOOD_CLOSE = 0.12;
    private static final double RIGHT_HOOD_FAR = 0.24;

    // Gate/Intake constants
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.5;
    private static final long INTAKE_DURATION_MS = 1000;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 200;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;

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

    // IMUs
    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;

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

        // LEDs (per-channel)
        LED led1Red = getLedSafe("led_1_red");
        LED led1Green = getLedSafe("led_1_green");
        LED led2Red = getLedSafe("led_2_red");
        LED led2Green = getLedSafe("led_2_green");

        // Voltage sensor (first available)
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
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

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

        String imuUsed = (pinpoint != null) ? "pinpoint" : (imu != null) ? "imu (expansion hub)" : "none";

        // Create controllers
        turretController = new TurretController(turret, imu, pinpoint, telemetry);
        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new FlywheelController(shooter, shooter2, telemetry, batterySensor); // pass voltage sensor
        flywheel.setShooterOn(false);

        gateController = new GateController(
                gateServo, intakeMotor,
                led1Red, led1Green, led2Red, led2Green,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQUENCE_POWER
        );

        clawController = new ClawController(clawServo, CLAW_OPEN, CLAW_CLOSED, CLAW_CLOSE_MS);
        hoodController = new HoodController(
                leftHoodServo, rightHoodServo,
                HOOD_MIN, RIGHT_HOOD_CLOSE,
                HOOD_MIN, HOOD_MAX,
                HOOD_LEFT_STEP, HOOD_RIGHT_STEP,
                HOOD_DEBOUNCE_MS
        );

        // Initial positions
        gateController.setGateClosed(true); // gate open => red on, green off, rn made it so that gate is closed at init
        telemetry.addData("Status", "Initialized (mode = CLOSE, shooter OFF)");
        telemetry.addData("RPM Switch Threshold", "%.0f RPM", FlywheelController.RPM_SWITCH_THRESHOLD);
        telemetry.addData("\nTurret IMU", imuUsed);

        telemetry.update();

        // Prepare subsystems
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

            // Touchpad reset (gamepad2)
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gamepad2TouchpadLast) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (pinpoint != null) {
                    try { pinpoint.resetPosAndIMU(); } catch (Exception ignored) {}
                } else if (imu != null) {
                    try { imu.initialize(imuParams); } catch (Exception ignored) {}
                }
                turretController.captureReferences();
                turretController.resetPidState();
                driveController.stop();
                telemetry.addData("Reset", "Heading ref and turret encoder zeroed");
                telemetry.update();
            }
            gamepad2TouchpadLast = gp2Touch;

            // A button reset (gamepad1) — mirrors touchpad reset
            boolean aNow = gamepad1.a;
            if (aNow && !aPressedLast) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (pinpoint != null) {
                    try { pinpoint.resetPosAndIMU(); } catch (Exception ignored) {}
                } else if (imu != null) {
                    try { imu.initialize(imuParams); } catch (Exception ignored) {}
                }
                turretController.captureReferences();
                turretController.resetPidState();
                driveController.stop();
                telemetry.addData("Reset", "Heading ref and turret encoder zeroed (gp1 A)");
                telemetry.update();
            }
            aPressedLast = aNow;

            // Far/close toggle on gamepad1 touchpad
            boolean touchpadNow = getTouchpad(gamepad1);
            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                // Hood automatically adjusts based on mode
                hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
            }
            touchpadPressedLast = touchpadNow;

            // Drive
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 1.0);

            // Flywheel toggles (DPAD)
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) flywheel.adjustTargetRPM(-50.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) flywheel.adjustTargetRPM(50.0);
            dpadRightLast = dpadRightNow;

            // Gate manual toggle (B)
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) {
                gateController.toggleGate();
            }
            bPressedLast = bNow;

            // Intake auto sequence (Y)
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy()) {
                gateController.startIntakeSequence(nowMs);
            }
            yPressedLast = yNow;

            // Gate sequence update + claw auto trigger
            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) {
                clawController.trigger(nowMs);
            }

            // Flywheel update
            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            // Check if PIDF mode changed and update hood accordingly
            boolean currentPidfMode = flywheel.isUsingFarCoefficients();
            if (currentPidfMode != lastPidfMode) {
                // PIDF mode switched, auto-adjust hood to match
                hoodController.setRightPosition(currentPidfMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                lastPidfMode = currentPidfMode;
            }

            // shooter2 now driven by FlywheelController; no manual mirroring needed

            // Rumble when at target
            if (flywheel.isAtTarget()) {
                final int RUMBLE_MS = 200;
                try { gamepad1.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
            }

            // Turret manual
            boolean manualNow = false;
            double manualPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.left_stick_x > 0.2) {
                manualNow = true; manualPower = 0.25;
            } else if (gamepad1.left_bumper || gamepad2.left_stick_x < -0.2) {
                manualNow = true; manualPower = -0.25;
            }
            turretController.update(manualNow, manualPower);

            // Intake manual (only if gate not busy)
            if (!gateController.isBusy()) {
                boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
                if (leftTriggerNow) {
                    intakeMotor.setPower(-1.0);
                } else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                    intakeMotor.setPower(1.0);
                } else {
                    intakeMotor.setPower(0.0);
                }
            }

            // Claw manual (X)
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawController.trigger(nowMs);
            }
            xPressedLast = xNow;
            clawController.update(nowMs);

            // Hood adjustments
            if (gamepad1.a) hoodController.nudgeLeftUp(nowMs);
            if (gamepad1.b) hoodController.nudgeLeftDown(nowMs);
            if (gamepad2.right_stick_y < -0.2) hoodController.nudgeRightUp(nowMs);
            else if (gamepad2.right_stick_y > 0.2) hoodController.nudgeRightDown(nowMs);

            // Telemetry: flywheel & gate with PIDF mode info
            String pidfModeStr = flywheel.isUsingFarCoefficients() ? "FAR" : "CLOSE";
            telemetry.addData("Flywheel", "Current: %.0f rpm | Target: %.0f rpm",
                    flywheel.getCurrentRPM(), flywheel.getTargetRPM());
            telemetry.addData("PIDF Mode", "%s (threshold: %.0f RPM)",
                    pidfModeStr, FlywheelController.RPM_SWITCH_THRESHOLD);
            telemetry.addData("PIDF Coefficients", "kP=%.5f kI=%.5f kD=%.6f kF=%.2f",
                    flywheel.isUsingFarCoefficients() ? FlywheelController.FAR_kP : FlywheelController.CLOSE_kP,
                    flywheel.isUsingFarCoefficients() ? FlywheelController.FAR_kI : FlywheelController.CLOSE_kI,
                    flywheel.isUsingFarCoefficients() ? FlywheelController.FAR_kD : FlywheelController.CLOSE_kD,
                    flywheel.isUsingFarCoefficients() ? FlywheelController.FAR_kF : FlywheelController.CLOSE_kF);
            telemetry.addData("Target Mode", isFarMode ? "FAR" : "CLOSE");
            telemetry.addData("\nGate", gateController.isGateClosed() ? "Closed" : "Open");
            telemetry.update();
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
        try {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretController.captureReferences();
            turretController.resetPidState();

            if (pinpoint != null) {
                pinpoint.resetPosAndIMU();
            } else if (imu != null) {
                imu.initialize(imuParams);
            }
        } catch (Exception ignored) {} // even if ignored we are resetting the limits at least
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretController.captureReferences();
        turretController.resetPidState();
    }
}