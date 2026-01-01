package org.firstinspires.ftc.teamcode.teleop;

/*
  secondexperimentalHORS.java
  ---------------------------
  - Gate servo toggles on B (gamepad1 or gamepad2) with LED indication (open = green, closed = red).
  - Y-button intake sequence: opens gate (if needed), runs intake, then closes gate and stops intake.
  - Shooter uses Flywheel subsystem logic (toggle on dpad down, left trigger low-RPM override, far/close modes via touchpad).
  - Shooter2 mirrors shooter power.
  - Rumble logic: continuous short rumbles while flywheel.isAtTarget().
  - Turret IMU preference: use "pinpoint" if present; otherwise "imu" (expansion hub).
*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

// subsystem imports (adjust package paths if yours differ)
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
    private static final double GATE_OPEN = 0.67; // lmao
    private static final double GATE_CLOSED = 0.5;

    // Gate + intake automation
    private static final long INTAKE_DURATION_MS = 1600; // sequence duration
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 500; // trigger claw 0.5s before sequence ends
    private static final double INTAKE_POWER = 0.92;              // manual intake
    private static final double INTAKE_SEQUENCE_POWER = 0.6;     // Y-button intake sequence power
    private enum GateCycleState { IDLE, OPEN_INTAKE }
    private GateCycleState gateCycleState = GateCycleState.IDLE;
    private boolean yPressedLast = false;
    private long gateActionStartMs = 0;
    private boolean intakeSequenceClawTriggered = false;

    // REV Digital LED Indicator (active-low) using hardware map names led1 (red) and led2 (green)
    private DigitalChannel ledLineRed;   // led1
    private DigitalChannel ledLineGreen; // led2

    // Subsystems
    private TurretController turretController;
    private DriveController driveController;
    private Flywheel flywheel;

    // UI / debounce and other small state
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
    private static final double RIGHT_HOOD_CLOSE   = 0.12;
    private static final double RIGHT_HOOD_FAR     = 0.24;

    // For gamepad2 touchpad reset
    private boolean gamepad2TouchpadLast = false;

    // IMUs
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
        // Gate servo (ensure hardware config uses the name "gateServo" or change accordingly)
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // REV Digital LED Indicator (single module, two DIO lines) using led1/led2
        try {
            ledLineRed = hardwareMap.get(DigitalChannel.class, "led1");   // assign red to led1
            ledLineGreen = hardwareMap.get(DigitalChannel.class, "led2"); // assign green to led2
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
        turret.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // mirror shooter mode
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
        flywheel = new Flywheel(shooter, telemetry);
        flywheel.setShooterOn(false); // keep shooter OFF during init

        // initial positions
        clawServo.setPosition(0.63);
        leftHoodServo.setPosition(leftHoodPosition);
        rightHoodPosition = RIGHT_HOOD_CLOSE;
        rightHoodServo.setPosition(rightHoodPosition);
        // Gate defaults to open
        gateClosed = false;
        gateServo.setPosition(GATE_OPEN);
        updateGateLed(); // reflect initial gate state

        String imuUsed = (turretImu == pinpointImu && pinpointImu != null) ? "pinpoint" :
                (turretImu == imu && imu != null) ? "imu (expansion hub)" : "none";
        telemetry.addData("Status", "Initialized (mode = CLOSE, shooter OFF)");
        telemetry.addData("Turret IMU", imuUsed);
        telemetry.update();

        // ensure subsystems are ready
        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();

        // Shooter default ON once TeleOp actually starts
        flywheel.setShooterOn(true);

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // ------------------------------
            // Touchpad toggles & reset
            // ------------------------------
            boolean touchpadNow = false;
            try { touchpadNow = gamepad1.touchpad; } catch (Throwable t) {
                touchpadNow = (gamepad1.left_stick_button && gamepad1.right_stick_button);
            }

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

            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                if (isFarMode) {
                    rightHoodPosition = RIGHT_HOOD_FAR;
                    rightHoodServo.setPosition(rightHoodPosition);
                } else {
                    rightHoodPosition = RIGHT_HOOD_CLOSE;
                    rightHoodServo.setPosition(rightHoodPosition);
                }
            }
            touchpadPressedLast = touchpadNow;

            // ------------------------------
            // DRIVE: delegate to DriveController
            // ------------------------------
            double axial   = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;
            double driveSpeed = 1.0;
            driveController.setDrive(axial, lateral, yaw, driveSpeed);

            // ------------------------------
            // DPAD shooter adjustments (Flywheel logic)
            // ------------------------------
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                flywheel.toggleShooterOn();
            }
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) {
                flywheel.adjustTargetRPM(-5.0);
            }
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) {
                flywheel.adjustTargetRPM(5.0);
            }
            dpadRightLast = dpadRightNow;

            // ------------------------------
            // B-button manual gate toggle (only when not in intake sequence)
            // ------------------------------
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && gateCycleState == GateCycleState.IDLE) {
                gateClosed = !gateClosed;
                gateServo.setPosition(gateClosed ? GATE_CLOSED : GATE_OPEN);
                updateGateLed();
            }
            bPressedLast = bNow;

            // ------------------------------
            // Y-button intake sequence: open gate, run intake at INTAKE_SEQUENCE_POWER,
            // trigger claw in last 0.5s, then close gate. No extra claw action at the end.
            // ------------------------------
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && gateCycleState == GateCycleState.IDLE) {
                gateClosed = false;
                gateServo.setPosition(GATE_OPEN);
                updateGateLed();
                intakeMotor.setPower(INTAKE_SEQUENCE_POWER); // use separate intake sequence power
                gateActionStartMs = nowMs;
                gateCycleState = GateCycleState.OPEN_INTAKE;
                intakeSequenceClawTriggered = false;
            }
            yPressedLast = yNow;

            if (gateCycleState == GateCycleState.OPEN_INTAKE) {
                long elapsedMs = nowMs - gateActionStartMs;
                long timeRemainingMs = INTAKE_DURATION_MS - elapsedMs;

                // Trigger claw action during the last 0.5 seconds (no additional claw action at end)
                if (timeRemainingMs <= CLAW_TRIGGER_BEFORE_END_MS && !intakeSequenceClawTriggered) {
                    clawServo.setPosition(0.2);
                    clawActionPhase = 1;
                    clawActionStartMs = nowMs;
                    intakeSequenceClawTriggered = true;
                }

                // End of intake sequence
                if (elapsedMs >= INTAKE_DURATION_MS) {
                    intakeMotor.setPower(0.0);
                    gateClosed = true;
                    gateServo.setPosition(GATE_CLOSED);
                    updateGateLed();
                    gateCycleState = GateCycleState.IDLE;
                }
            }

            // ------------------------------
            // Flywheel update (measurement + PID + motor write)
            // ------------------------------
            boolean calibPressed = gamepad1.back || gamepad2.back; // optional calibration trigger
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            // Mirror shooter power to shooter2 (opposite direction via motor config)
            shooter2.setPower(shooter.getPower());

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
                manualPower = 0.25;
            } else if (gamepad1.left_bumper || gamepad2.left_stick_x < -0.2) {
                manualNow = true;
                manualPower = -0.25;
            }
            turretController.update(manualNow, manualPower);

            // ------------------------------
            // INTAKE manual control (only when not in auto sequence)
            // ------------------------------
            if (gateCycleState == GateCycleState.IDLE) {
                boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
                if (leftTriggerNow) {
                    intakeMotor.setPower(-1.0);
                } else {
                    if ((gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                        intakeMotor.setPower(1.0);
                    } else {
                        intakeMotor.setPower(0.0);
                    }
                }
            }

            // CLAW toggle (X)
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

            // Hood adjustments (A/B kept for hood adjust; gate toggle moved to B above)
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
            telemetry.addData("Mode", isFarMode ? "FAR" : "CLOSE");
            telemetry.addData("Turret Enc", turret.getCurrentPosition());
            telemetry.addData("Turret Power (applied)", turretController.getLastAppliedPower());
            telemetry.addData("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
            telemetry.addData("Fly Target", String.format("%.1f", flywheel.getTargetRPM()));
            telemetry.addData("Fly AtTarget", flywheel.isAtTarget());
            telemetry.addData("Gate", gateClosed ? "CLOSED" : "OPEN");
            telemetry.addData("Gate Pos", gateServo.getPosition());
            telemetry.addData("Gate Cycle", gateCycleState);

            String imuUsedNow = (turretImu == pinpointImu && pinpointImu != null) ? "pinpoint" :
                    (turretImu == imu && imu != null) ? "imu (exp hub)" : "none";
            telemetry.addData("Turret IMU Used", imuUsedNow);

            telemetry.update();
        }
    }

    private void headingReferenceReset() {
        // turretController.captureReferences() handles the turret mapping if needed.
    }

    // Active-low LED helper: gate open -> green, gate closed -> red
    private void updateGateLed() {
        if (ledLineRed == null || ledLineGreen == null) return;
        // off = true, on = false (active-low)
        if (gateClosed) {
            // gate closed -> RED ON, GREEN OFF
            ledLineRed.setState(false);
            ledLineGreen.setState(true);
        } else {
            // gate open -> GREEN ON, RED OFF
            ledLineRed.setState(true);
            ledLineGreen.setState(false);
        }
    }
}