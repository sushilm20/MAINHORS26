package org.firstinspires.ftc.teamcode.teleop;

/*
  HORS EXPERIMENTAL
  Turret prefers Pinpoint IMU; falls back to expansion-hub IMU.
*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.TurretController;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

@TeleOp(name="HORS EXPERIMENTAL 🤖", group="Linear OpMode")
public class wildexperiment extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;

    // Gate servo
    private Servo gateServo;
    private boolean gateClosed = false;
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.5;

    // Gate + intake automation
    private static final long INTAKE_DURATION_MS = 1600;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 500;
    private static final double INTAKE_POWER = 0.92;
    private static final double INTAKE_SEQUENCE_POWER = 0.6;

    private enum GateCycleState {IDLE, OPEN_INTAKE}
    private GateCycleState gateCycleState = GateCycleState.IDLE;
    private boolean yPressedLast = false;
    private long gateActionStartMs = 0;
    private boolean intakeSequenceClawTriggered = false;

    private DigitalChannel ledLineRed;
    private DigitalChannel ledLineGreen;

    // Subsystems
    private TurretController turretController;
    private DriveController driveController;
    private Flywheel flywheel;

    private TelemetryManager panelsTelemetry;

    // UI state
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean xPressedLast = false;
    private boolean bPressedLast = false;

    // hood/claw
    private double leftHoodPosition = 0.12;
    private double rightHoodPosition = 0.12;
    private long lastLeftHoodAdjustMs = 0L;
    private long lastRightHoodAdjustMs = 0L;
    private static final long HOOD_ADJUST_DEBOUNCE_MS = 120L;

    private int clawActionPhase = 0;
    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 500L;

    // Far/Close mode
    private boolean isFarMode = false;
    private boolean touchpadPressedLast = false;
    private static final double RIGHT_HOOD_CLOSE = 0.12;
    private static final double RIGHT_HOOD_FAR = 0.24;

    private boolean gamepad2TouchpadLast = false;

    // IMUs
    private BNO055IMU imu;                 // expansion hub IMU (named "imu" in config)
    private GoBildaPinpointDriver pinpoint; // Pinpoint (named "pinpoint" in config)

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        turret.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU init
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
        } catch (Exception e) { imu = null; }

        // Pinpoint init
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.resetPosAndIMU();
        } catch (Exception e) { pinpoint = null; }

        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        if (imu != null) {
            try { imu.initialize(imuParams); } catch (Exception ignored) {}
        }

        String imuUsed;
        if (pinpoint != null) {
            imuUsed = "pinpoint";
        } else if (imu != null) {
            imuUsed = "imu (expansion hub)";
        } else {
            imuUsed = "none";
        }

        turretController = new TurretController(turret, imu, pinpoint, telemetry);
        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new Flywheel(shooter, telemetry);
        flywheel.setShooterOn(false);

        clawServo.setPosition(0.63);
        leftHoodServo.setPosition(leftHoodPosition);
        rightHoodPosition = RIGHT_HOOD_CLOSE;
        rightHoodServo.setPosition(rightHoodPosition);
        gateClosed = false;
        gateServo.setPosition(GATE_OPEN);

        telemetry.addData("Status", "Initialized (mode = CLOSE, shooter OFF)");
        telemetry.addData("Turret IMU", imuUsed);
        telemetry.update();

        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();
        flywheel.setShooterOn(true);

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // Refresh Pinpoint each loop
            if (pinpoint != null) {
                try { pinpoint.update(); } catch (Exception ignored) {}
            }

            // Touchpad toggles & reset
            boolean touchpadNow = false;
            try { touchpadNow = gamepad1.touchpad; }
            catch (Throwable t) { touchpadNow = (gamepad1.left_stick_button && gamepad1.right_stick_button); }

            boolean gamepad2TouchpadNow = false;
            try { gamepad2TouchpadNow = gamepad2.touchpad; }
            catch (Throwable t) { gamepad2TouchpadNow = (gamepad2.left_stick_button && gamepad2.right_stick_button); }

            if (gamepad2TouchpadNow && !gamepad2TouchpadLast) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (pinpoint != null) {
                    try { pinpoint.resetPosAndIMU(); } catch (Exception ignored) {}
                }
                turretController.captureReferences();
                turretController.resetPidState();
                driveController.stop();
                telemetry.addData("Reset", "Heading reference and turret encoder set to zero!");
                telemetry.update();
            }
            gamepad2TouchpadLast = gamepad2TouchpadNow;

            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                rightHoodPosition = isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE;
                rightHoodServo.setPosition(rightHoodPosition);
            }
            touchpadPressedLast = touchpadNow;

            // Drive
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 1.0);

            // Shooter controls
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) flywheel.adjustTargetRPM(-5.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) flywheel.adjustTargetRPM(5.0);
            dpadRightLast = dpadRightNow;

            // Gate toggle (B)
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && gateCycleState == GateCycleState.IDLE) {
                gateClosed = !gateClosed;
                gateServo.setPosition(gateClosed ? GATE_CLOSED : GATE_OPEN);
            }
            bPressedLast = bNow;

            // Intake sequence (Y)
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && gateCycleState == GateCycleState.IDLE) {
                gateClosed = false;
                gateServo.setPosition(GATE_OPEN);
                intakeMotor.setPower(INTAKE_SEQUENCE_POWER);
                gateActionStartMs = nowMs;
                gateCycleState = GateCycleState.OPEN_INTAKE;
                intakeSequenceClawTriggered = false;
            }
            yPressedLast = yNow;

            if (gateCycleState == GateCycleState.OPEN_INTAKE) {
                long elapsedMs = nowMs - gateActionStartMs;
                long timeRemainingMs = INTAKE_DURATION_MS - elapsedMs;

                if (timeRemainingMs <= CLAW_TRIGGER_BEFORE_END_MS && !intakeSequenceClawTriggered) {
                    clawServo.setPosition(0.2);
                    clawActionPhase = 1;
                    clawActionStartMs = nowMs;
                    intakeSequenceClawTriggered = true;
                }

                if (elapsedMs >= INTAKE_DURATION_MS) {
                    intakeMotor.setPower(0.0);
                    gateClosed = true;
                    gateServo.setPosition(GATE_CLOSED);
                    gateCycleState = GateCycleState.IDLE;
                }
            }

            // Flywheel update
            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);
            shooter2.setPower(shooter.getPower());

            // Rumble when at target
            if (flywheel.isAtTarget()) {
                final int RUMBLE_MS = 200;
                try { gamepad1.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
            }

            // Turret manual control
            boolean manualNow = false;
            double manualPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.left_stick_x > 0.2) {
                manualNow = true;
                manualPower = 0.25;
            } else if (gamepad1.left_bumper || gamepad2.left_stick_x < -0.2) {
                manualNow = true;
                manualPower = -0.25;
            }
            turretController.update(manualNow, manualPower);

            // Intake manual control (when not in sequence)
            if (gateCycleState == GateCycleState.IDLE) {
                boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
                if (leftTriggerNow) {
                    intakeMotor.setPower(-1.0);
                } else if ((gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                    intakeMotor.setPower(1.0);
                } else {
                    intakeMotor.setPower(0.0);
                }
            }

            // Claw toggle (X)
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
            telemetry.addData("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
            telemetry.addData("Fly Target", String.format("%.1f", flywheel.getTargetRPM()));
            telemetry.addData("Gate", gateClosed ? "CLOSED" : "OPEN");
            telemetry.addData("Gate Cycle", gateCycleState);
            String imuUsedNow = (pinpoint != null) ? "pinpoint" :
                    (imu != null) ? "imu (exp hub)" : "none";
            telemetry.addData("Turret IMU Used", imuUsedNow);
            telemetry.update();
        }
    }

    private void headingReferenceReset() {
        // turretController.captureReferences() handles the turret mapping if needed.
    }
}