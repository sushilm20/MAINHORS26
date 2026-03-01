package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
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

import org.firstinspires.ftc.teamcode.extras.TelemetryData;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.tracking.FieldOrientedTurretController;

import java.util.List;

@TeleOp(name = "V2 Dynamic Turret Tracking 🎯🗺️", group = "Linear OpMode")
public class DynamicTrackingV2 extends LinearOpMode {

    // Drive + subsystems
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;
    private DigitalChannel turretLimitSwitch;

    // NEW: FieldOrientedTurretController instead of TurretController
    private FieldOrientedTurretController fieldTurret;
    private DriveController driveController;
    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;

    // Telemetry
    private TelemetryManager panelsTelemetry;
    private TelemetryData telemetryData;

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
    private boolean lastPidfMode = false;

    // Hood presets
    private static final double RIGHT_HOOD_CLOSE = 0.16;
    private static final double RIGHT_HOOD_FAR = 0.26;

    // Gate/Intake constants
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.485;
    private static final long INTAKE_DURATION_MS = 1050;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 400;
    private static final double INTKE_SEQUENCE_POWER = 1.0;

    // Hood constants
    private static final double HOOD_MIN = 0.12;
    private static final double HOOD_MAX = 0.45;
    private static final double HOOD_LEFT_STEP = 0.025;
    private static final double HOOD_RIGHT_STEP = 0.01;
    private static final long HOOD_DEBOUNCE_MS = 120L;

    // Pinpoint IMU only
    private GoBildaPinpointDriver pinpoint;

    // Pose tracking via Pedro Pathing
    private Follower follower;
    private Pose currentPose = new Pose();

    // Starting pose (robot starts here with turret encoder at 0)
    private static final double START_X = 22.0;
    private static final double START_Y = 122.0;
    private static final double START_HEADING_DEG = 135.0;

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // ════════════ BULK READ — must be first! ════════════
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

        // Optional: turret limit switch (active-low)
        try {
            turretLimitSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
            turretLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            turretLimitSwitch = null;
        }

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
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pinpoint IMU init (required — no fallback)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        // Pedro Pathing follower for pose tracking
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        } catch (Exception e) {
            telemetry.addData("Error", "Follower initialization failed!");
            follower = null;
        }

        // ════════════ NEW: FieldOrientedTurretController ════════════
        fieldTurret = new FieldOrientedTurretController(turret, pinpoint, telemetry);
        if (turretLimitSwitch != null) {
            fieldTurret.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }
        // Goal defaults to (14, 134) — change via dashboard or here:
        // fieldTurret.setGoal(14.0, 134.0);

        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new FlywheelController(shooter, shooter2, telemetry, batterySensor);
        flywheel.setShooterOn(false);

        telemetryData = new TelemetryData(telemetry, flywheel);

        gateController = new GateController(
                gateServo, intakeMotor,
                led1Red, led1Green, led2Red, led2Green,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTKE_SEQUENCE_POWER
        );

        clawController = new ClawController(clawServo, ClawController.CLAW_OPEN, ClawController.CLAW_CLOSED, ClawController.CLAW_CLOSE_MS);
        hoodController = new HoodController(
                leftHoodServo, rightHoodServo,
                HOOD_MIN, RIGHT_HOOD_CLOSE,
                HOOD_MIN, HOOD_MAX,
                HOOD_LEFT_STEP, HOOD_RIGHT_STEP,
                HOOD_DEBOUNCE_MS
        );

        // Initial positions
        gateController.setGateClosed(true);
        telemetry.addData("Status", "Initialized (mode = CLOSE, shooter OFF)");
        telemetry.addData("RPM Switch Threshold", "%.0f RPM", FlywheelController.RPM_SWITCH_THRESHOLD);
        telemetry.addData("Bulk Read", "AUTO");
        telemetry.addData("Turret Mode", "FIELD-ORIENTED (pose + heading unified)");
        telemetry.addData("Goal", "(%.0f, %.0f)", FieldOrientedTurretController.GOAL_X, FieldOrientedTurretController.GOAL_Y);
        telemetry.update();

        // Prepare subsystems
        fieldTurret.captureReferences();
        fieldTurret.resetPidState();

        waitForStart();

        if (isStopRequested()) {
            fieldTurret.disable();
            return;
        }

        // After start: re-zero turret and heading
        reZeroHeadingAndTurret();
        flywheel.setShooterOn(true);

        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            // ── Update loop time ──
            telemetryData.updateLoopTime();

            // Refresh pinpoint
            try { pinpoint.update(); } catch (Exception ignored) {}

            // Update follower and get pose — critical for field-oriented aiming
            if (follower != null) {
                follower.update();
                currentPose = follower.getPose();
            }

            // ════════════ RESET BUTTONS ════════════

            // Touchpad reset (gamepad2)
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gamepad2TouchpadLast) {
                resetTurretEncoderAndReferences();
                driveController.stop();
                telemetry.addData("Reset", "Turret encoder reset & reference captured (gp2 touchpad)");
                telemetry.update();
            }
            gamepad2TouchpadLast = gp2Touch;

            // A button reset (gamepad1) — resets turret encoder to 0 and recaptures
            boolean aNow = gamepad1.a;
            if (aNow && !aPressedLast) {
                resetTurretEncoderAndReferences();
                driveController.stop();
                telemetry.addData("Reset", "Turret encoder reset & reference captured (gp1 A)");
                telemetry.update();
            }
            aPressedLast = aNow;

            // Far/close toggle on gamepad1 touchpad only
            boolean touchpadNow = getTouchpad(gamepad1);
            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
            }
            touchpadPressedLast = touchpadNow;

            // ════════════ DRIVE ════════════
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 1.0);

            // ════════════ FLYWHEEL ════════════
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) flywheel.adjustTargetRPM(-50.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) flywheel.adjustTargetRPM(50.0);
            dpadRightLast = dpadRightNow;

            // ════════════ GATE ════════════
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
                hoodController.setRightPosition(currentPidfMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                lastPidfMode = currentPidfMode;
            }

            // Rumble when at target
            if (flywheel.isAtTarget()) {
                final int RUMBLE_MS = 200;
                try { gamepad1.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
            }

            // ════════════ TURRET (FIELD-ORIENTED) ════════════
            fieldTurret.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);

            boolean manualNow = false;
            double manualPower = 0.0;

            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) {
                manualNow = true; manualPower = 0.35;
            } else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) {
                manualNow = true; manualPower = -0.35;
            }

            // Pass currentPose — when available, turret aims at goal using field math
            // When null (follower failed), falls back to heading-hold
            fieldTurret.update(manualNow, manualPower, currentPose);

            // ════════════ INTAKE MANUAL ════════════
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

            // ════════════ CLAW (X) ════════════
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawController.trigger(nowMs);
            }
            xPressedLast = xNow;
            clawController.update(nowMs);

            // ════════════ HOOD ════════════
            if (gamepad1.a) hoodController.nudgeLeftUp(nowMs);
            if (gamepad1.b) hoodController.nudgeLeftDown(nowMs);

            // ════════════ TELEMETRY ════════════
            telemetryData.setPose(currentPose);

            telemetry.addData("Turret.Bearing", "%.1f°", fieldTurret.getLastBearingDeg());
            telemetry.addData("Turret.Dist", "%.1f in", fieldTurret.getLastDistToGoal());
            telemetry.addData("Turret.Mode", fieldTurret.isFreezeMode() ? "FREEZE 🔒" :
                    (fieldTurret.isHomingMode() ? "HOMING 🔄" : "FIELD AIM 🎯"));

            telemetryData.update();
        }

        fieldTurret.disable();
    }

    // ════════════ HELPERS ════════════

    private boolean getTouchpad(com.qualcomm.robotcore.hardware.Gamepad gp) {
        try { return gp.touchpad; }
        catch (Throwable t) { return gp.left_stick_button && gp.right_stick_button; }
    }

    private LED getLedSafe(String name) {
        try { return hardwareMap.get(LED.class, name); }
        catch (Exception ignored) { return null; }
    }

    private void reZeroHeadingAndTurret() {
        try { pinpoint.update(); } catch (Exception ignored) {}
        fieldTurret.captureReferences();
        fieldTurret.resetPidState();
    }

    /**
     * Reset turret encoder to 0, recapture heading reference, clear PID.
     * Triggered by gamepad1 A or gamepad2 touchpad.
     */
    private void resetTurretEncoderAndReferences() {
        try { pinpoint.update(); } catch (Exception ignored) {}
        fieldTurret.recenterAndResume(true);
    }
}