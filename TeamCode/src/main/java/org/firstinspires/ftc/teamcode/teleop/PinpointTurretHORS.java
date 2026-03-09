package org.firstinspires.ftc.teamcode.teleop;

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

import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.tracking.BearingTurretController;

import java.util.List;

@TeleOp(name="A HORS TURRET 🎯", group="Linear OpMode")
public class PinpointTurretHORS extends LinearOpMode {

    // Drive + subsystems
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;
    private DigitalChannel turretLimitSwitch;

    private BearingTurretController bearingTurret;
    private DriveController driveController;
    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;

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

    // ── NEW: turret starts locked (frozen) until first A press ──
    private boolean turretUnlocked = false;

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
    private static final double HOOD_MAX = 0.5;
    private static final double HOOD_LEFT_STEP = 0.025;
    private static final double HOOD_RIGHT_STEP = 0.01;
    private static final long HOOD_DEBOUNCE_MS = 120L;

    // Starting pose constants
    private static final double START_X = 20;
    private static final double START_Y = 122;
    private static final double START_HEADING_DEG = 135;

    // Pose tracking
    private Follower follower;
    private Pose currentPose = new Pose();

    // Loop timing
    private long lastLoopMs = 0;
    private double loopTimeMs = 0;

    @Override
    public void runOpMode() {

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

        // Follower init (Pedro Pathing — sole source of pose + heading)
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        } catch (Exception e) {
            telemetry.addData("Error", "Follower initialization failed!");
            follower = null;
        }

        // Create BearingTurretController — uses Follower, no IMU needed
        bearingTurret = new BearingTurretController(turret, follower, telemetry);
        bearingTurret.setGoal(13, 135);
        if (turretLimitSwitch != null) {
            bearingTurret.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

        // ── NEW: freeze turret immediately at init ──
        bearingTurret.freeze();

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
        telemetry.addData("Status", "Initialized (Bearing Turret) — TURRET FROZEN");
        telemetry.addData("Goal", "(%.0f, %.0f)", BearingTurretController.GOAL_X, BearingTurretController.GOAL_Y);
        telemetry.addData("Turret", "FROZEN — press A after start to unlock");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            bearingTurret.disable();
            return;
        }

        // After start: zero turret encoder and start flywheel
        // Turret stays FROZEN — will not track until A is pressed
        bearingTurret.zeroEncoder();
        bearingTurret.freeze();          // ── NEW: ensure still frozen after start ──
        turretUnlocked = false;          // ── NEW: explicitly mark as locked ──
        flywheel.setShooterOn(true);
        lastLoopMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            // ── Loop timing ──
            loopTimeMs = nowMs - lastLoopMs;
            lastLoopMs = nowMs;

            // ── Update follower (MUST be before turret update) ──
            if (follower != null) {
                follower.update();
                currentPose = follower.getPose();
            }

            // ── Touchpad reset (gamepad2): zero turret encoder + clear offset ──
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gamepad2TouchpadLast) {
                bearingTurret.zeroEncoder();
                bearingTurret.clearManualOffset();
                driveController.stop();
            }
            gamepad2TouchpadLast = gp2Touch;

            // ══════════════════════════════════════════════════
            //  A BUTTON — Three-mode logic:
            //    If turret has never been unlocked:
            //      → zero encoder, clear offset, reset pose, unfreeze, unlock
            //    Otherwise original two-press toggle:
            //      1st press (tracking): home encoder + freeze
            //      2nd press (frozen):   reset pose + unfreeze + resume tracking
            // ══════════════════════════════════════════════════
            boolean aNow = gamepad1.a;
            if (aNow && !aPressedLast) {
                if (!turretUnlocked) {
                    // ── NEW: First-ever A press — unlock the turret ──
                    bearingTurret.zeroEncoder();
                    bearingTurret.clearManualOffset();
                    if (follower != null) {
                        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
                    }
                    bearingTurret.unfreeze();
                    turretUnlocked = true;
                } else if (!bearingTurret.isFreezeMode()) {
                    // Already unlocked, currently tracking → freeze
                    bearingTurret.zeroEncoder();
                    bearingTurret.clearManualOffset();
                    bearingTurret.freeze();
                    driveController.stop();
                } else {
                    // Already unlocked, currently frozen → reset pose & unfreeze
                    if (follower != null) {
                        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
                    }
                    bearingTurret.clearManualOffset();
                    bearingTurret.unfreeze();
                }
            }
            aPressedLast = aNow;

            // ── Far/close toggle on gamepad1 touchpad ──
            boolean touchpadNow = getTouchpad(gamepad1);
            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
            }
            touchpadPressedLast = touchpadNow;

            // ── Drive ──
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 1.0);

            // ── Flywheel toggles (DPAD) ──
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) flywheel.adjustTargetRPM(-50.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) flywheel.adjustTargetRPM(50.0);
            dpadRightLast = dpadRightNow;

            // ── Gate manual toggle (B) ──
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) {
                gateController.toggleGate();
            }
            bPressedLast = bNow;

            // ── Intake auto sequence (Y) ──
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy()) {
                gateController.startIntakeSequence(nowMs);
            }
            yPressedLast = yNow;

            // ── Gate sequence update + claw auto trigger ──
            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) {
                clawController.trigger(nowMs);
            }

            // ── Flywheel update ──
            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            // ── Hood auto-switch on PIDF mode change ──
            boolean currentPidfMode = flywheel.isUsingFarCoefficients();
            if (currentPidfMode != lastPidfMode) {
                hoodController.setRightPosition(currentPidfMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                lastPidfMode = currentPidfMode;
            }

            // ── Rumble when at target ──
            if (flywheel.isAtTarget()) {
                final int RUMBLE_MS = 200;
                try { gamepad1.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
            }

            // ── Turret: homing sweep (dpad_up / gp2 left bumper) ──
            bearingTurret.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);

            // ── Turret: manual override (bumpers / gp2 right stick) ──
            boolean manualNow = false;
            double manualPower = 0.0;

            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) {
                manualNow = true; manualPower = 0.35;
            } else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) {
                manualNow = true; manualPower = -0.35;
            }

            // ── Turret update (bearing-based auto aim) ──
            bearingTurret.update(manualNow, manualPower);

            // ── Intake manual (only if gate not busy) ──
            if (!gateController.isBusy()) {
                boolean gp1LeftTrigger = gamepad1.left_trigger > 0.1;
                boolean gp2LeftTrigger = gamepad2.left_trigger > 0.1;
                boolean gp1RightTrigger = gamepad1.right_trigger > 0.1;
                boolean gp2RightTrigger = gamepad2.right_trigger > 0.1;

                if (gp1LeftTrigger || gp2LeftTrigger) {
                    intakeMotor.setPower(-1.0);
                } else if (gp1RightTrigger || gp2RightTrigger) {
                    // In far mode, gamepad1 intake runs at half power;
                    // gamepad2 always runs at full power.
                    if (isFarMode && gp1RightTrigger && !gp2RightTrigger) {
                        intakeMotor.setPower(0.7);
                    } else {
                        // gp2 is pressing (full power always), or close/default mode (full power)
                        intakeMotor.setPower(1.0);
                    }
                } else {
                    intakeMotor.setPower(0.0);
                }
            }

            // ── Claw manual (X) ──
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawController.trigger(nowMs);
            }
            xPressedLast = xNow;
            clawController.update(nowMs);

            // ── Hood adjustments ──
            if (gamepad1.a) hoodController.nudgeLeftUp(nowMs);
            if (gamepad1.b) hoodController.nudgeLeftDown(nowMs);

            // ════════════════════════════════════════════
            //  MINIMAL TELEMETRY
            // ════════════════════════════════════════════

            // Pose
            telemetry.addData("Pose", "(%.1f, %.1f) %.1f°",
                    currentPose.getX(), currentPose.getY(),
                    Math.toDegrees(currentPose.getHeading()));

            // Flywheel
            telemetry.addData("Flywheel", "%.0f / %.0f rpm %s",
                    flywheel.getCurrentRPM(),
                    flywheel.getTargetRPM(),
                    flywheel.isAtTarget() ? "✓" : "");

            // Turret offset + mode (updated to show LOCKED state)
            telemetry.addData("Turret Offset", "%.1f°", bearingTurret.getManualOffsetDeg());
            telemetry.addData("Turret Mode",
                    !turretUnlocked ? "LOCKED (press A to unlock)" :
                            bearingTurret.isFreezeMode() ? "FROZEN (press A to reset)" :
                                    bearingTurret.isHomingMode() ? "HOMING" : "TRACKING");

            // Loop
            telemetry.addData("Loop", "%.0f ms (%.0f Hz)",
                    loopTimeMs,
                    loopTimeMs > 0 ? 1000.0 / loopTimeMs : 0);

            telemetry.update();
        }

        bearingTurret.disable();
    }

    private boolean getTouchpad(com.qualcomm.robotcore.hardware.Gamepad gp) {
        try { return gp.touchpad; }
        catch (Throwable t) { return gp.left_stick_button && gp.right_stick_button; }
    }

    private LED getLedSafe(String name) {
        try { return hardwareMap.get(LED.class, name); }
        catch (Exception ignored) { return null; }
    }
}