package org.firstinspires.ftc.teamcode.teleop.AdaptiveHORS;

import com.bylazar.configurables.annotations.Configurable;
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
import org.firstinspires.ftc.teamcode.subsystems.AutoShooter.FlywheelVersatile;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.subsystems.AutoShooter.ShooterCalibration;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

/**
 * AdaptiveHORS Blue — pose-based auto-adjusting RPM using
 * {@link FlywheelVersatile} and {@link ShooterCalibration}.
 *
 * <ul>
 *   <li><b>dpad left/right</b> — trim RPM ±50 on top of auto RPM</li>
 *   <li><b>back button</b> — re-run regression live (after Panels edits)</li>
 *   <li><b>touchpad (gp1)</b> — toggle far/close hood preset</li>
 *   <li><b>dpad down</b> — toggle shooter on/off</li>
 * </ul>
 */
@Configurable
@TeleOp(name = "ADAPTIVE HORS 🧨", group = "Linear OpMode")
public class AdaptiveHORSBlue extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────────
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo, leftHoodServo, rightHoodServo, gateServo;
    private DigitalChannel turretLimitSwitch;

    // ── Controllers ──────────────────────────────────────────
    private TurretController   turretController;
    private DriveController    driveController;
    private FlywheelController flywheel;
    private FlywheelVersatile  flywheelVersatile;
    private GateController     gateController;
    private ClawController     clawController;
    private HoodController     hoodController;

    // ── Telemetry ────────────────────────────────────────────
    private TelemetryManager panelsTelemetry;

    // ── Toggle / debounce state ──────────────────────────────
    private boolean dpadDownLast        = false;
    private boolean dpadLeftLast        = false;
    private boolean dpadRightLast       = false;
    private boolean dpadUpLast          = false;
    private boolean xPressedLast        = false;
    private boolean bPressedLast        = false;
    private boolean yPressedLast        = false;
    private boolean touchpadPressedLast = false;
    private boolean gamepad2TouchpadLast = false;
    private boolean aPressedLast        = false;
    private boolean backPressedLast     = false;

    private boolean isFarMode    = false;
    private boolean lastPidfMode = false;

    // ── Hood presets ─────────────────────────────────────────
    private static final double RIGHT_HOOD_CLOSE = 0.16;
    private static final double RIGHT_HOOD_FAR   = 0.24;

    // ── Gate / Intake ────────────────────────────────────────
    private static final double GATE_OPEN              = 0.67;
    private static final double GATE_CLOSED            = 0.485;
    private static final long   INTAKE_DURATION_MS     = 1050;
    private static final long   CLAW_TRIGGER_BEFORE_END_MS = 400;
    private static final double INTAKE_SEQUENCE_POWER  = 1.0;

    // ── Hood ─────────────────────────────────────────────────
    private static final double HOOD_MIN        = 0.12;
    private static final double HOOD_MAX        = 0.45;
    private static final double HOOD_LEFT_STEP  = 0.025;
    private static final double HOOD_RIGHT_STEP = 0.01;
    private static final long   HOOD_DEBOUNCE_MS = 120L;

    // ── Pinpoint / Pose ──────────────────────────────────────
    private GoBildaPinpointDriver pinpoint;
    private Follower follower;

    // ── Blue-side default calibration values (canonical source of truth) ──
    private static final double DEFAULT_GOAL_X = 15.0;
    private static final double DEFAULT_GOAL_Y = 135.0;
    private static final double DEFAULT_CAL1_X = 48.0;  private static final double DEFAULT_CAL1_Y = 96.0;
    private static final double DEFAULT_CAL2_X = 60.0;  private static final double DEFAULT_CAL2_Y = 125.0;
    private static final double DEFAULT_CAL3_X = 60.0;  private static final double DEFAULT_CAL3_Y = 82.0;
    private static final double DEFAULT_CAL4_X = 72.0;  private static final double DEFAULT_CAL4_Y = 72.0;
    private static final double DEFAULT_CAL5_X = 72.0;  private static final double DEFAULT_CAL5_Y = 120.0;
    private static final double DEFAULT_CAL6_X = 95.0;  private static final double DEFAULT_CAL6_Y = 120.0;
    private static final double DEFAULT_CAL7_X = 52.0;  private static final double DEFAULT_CAL7_Y = 14.0;

    private static final Pose START_POSE = new Pose(20, 122, Math.toRadians(135));

    // FIX: Initialize to START_POSE so first-frame RPM calc uses real position
    private Pose currentPose = START_POSE;

    // Blue goal target pose (only x, y matter for distance)
    private static final Pose BLUE_GOAL = new Pose(12, 135, 0);

    // FIX: Guard against localizer returning (0,0) before it has a real fix
    private boolean followerPoseValid = false;

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // ════════════ Hardware map ��═══════════
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRight");

        shooter     = hardwareMap.get(DcMotor.class, "shooter");
        shooter2    = hardwareMap.get(DcMotor.class, "shooter2");
        turret      = hardwareMap.get(DcMotor.class, "turret");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        clawServo      = hardwareMap.get(Servo.class, "clawServo");
        leftHoodServo  = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        gateServo      = hardwareMap.get(Servo.class, "gateServo");

        try {
            turretLimitSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
            turretLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) { turretLimitSwitch = null; }

        LED led1Red   = getLedSafe("led_1_red");
        LED led1Green = getLedSafe("led_1_green");
        LED led2Red   = getLedSafe("led_2_red");
        LED led2Green = getLedSafe("led_2_green");

        VoltageSensor batterySensor = null;
        try { batterySensor = hardwareMap.voltageSensor.iterator().next(); }
        catch (Exception ignored) {}

        // ════════════ Motor directions ════════════
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

        // ════════════ Pinpoint ════════════
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        // ════════════ Follower ════════════
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(START_POSE);
        } catch (Exception e) {
            telemetry.addData("Error", "Follower init failed!");
            follower = null;
        }

        // ════════════ Flywheel + Adaptive RPM ════════════
        flywheel = new FlywheelController(shooter, shooter2, telemetry, batterySensor);
        flywheel.setShooterOn(false);

        // FIX: Restore blue-side calibration statics in case Red ran previously
        //      and mutated the shared static fields.
        ShooterCalibration.GOAL_X = DEFAULT_GOAL_X;
        ShooterCalibration.GOAL_Y = DEFAULT_GOAL_Y;
        ShooterCalibration.CAL1_X = DEFAULT_CAL1_X;  ShooterCalibration.CAL1_Y = DEFAULT_CAL1_Y;
        ShooterCalibration.CAL2_X = DEFAULT_CAL2_X;  ShooterCalibration.CAL2_Y = DEFAULT_CAL2_Y;
        ShooterCalibration.CAL3_X = DEFAULT_CAL3_X;  ShooterCalibration.CAL3_Y = DEFAULT_CAL3_Y;
        ShooterCalibration.CAL4_X = DEFAULT_CAL4_X;  ShooterCalibration.CAL4_Y = DEFAULT_CAL4_Y;
        ShooterCalibration.CAL5_X = DEFAULT_CAL5_X;  ShooterCalibration.CAL5_Y = DEFAULT_CAL5_Y;
        ShooterCalibration.CAL6_X = DEFAULT_CAL6_X;  ShooterCalibration.CAL6_Y = DEFAULT_CAL6_Y;
        ShooterCalibration.CAL7_X = DEFAULT_CAL7_X;  ShooterCalibration.CAL7_Y = DEFAULT_CAL7_Y;

        ShooterCalibration calibration = new ShooterCalibration();
        calibration.computeRegression();

        flywheelVersatile = new FlywheelVersatile(
                flywheel, BLUE_GOAL, calibration, 2300, 4000
        );

        // ════════════ Other controllers ════════════
        turretController = new TurretController(turret, null, pinpoint, telemetry);
        if (turretLimitSwitch != null) {
            turretController.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

        driveController = new DriveController(
                frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        gateController = new GateController(
                gateServo, intakeMotor,
                led1Red, led1Green, led2Red, led2Green,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQUENCE_POWER
        );

        clawController = new ClawController(
                clawServo, ClawController.CLAW_OPEN,
                ClawController.CLAW_CLOSED, ClawController.CLAW_CLOSE_MS);

        hoodController = new HoodController(
                leftHoodServo, rightHoodServo,
                HOOD_MIN, RIGHT_HOOD_CLOSE,
                HOOD_MIN, HOOD_MAX,
                HOOD_LEFT_STEP, HOOD_RIGHT_STEP,
                HOOD_DEBOUNCE_MS
        );

        // ════════════ Initial state ════════════
        gateController.setGateClosed(true);

        // FIX: Use calibration's distanceToGoal so distance is computed
        //      against the same goal used by the regression.
        double startDist = ShooterCalibration.distanceToGoal(
                START_POSE.getX(), START_POSE.getY());
        telemetry.addData("Status", "Initialized — Adaptive RPM (shooter OFF)");
        telemetry.addData("Start→Goal dist", "%.1f in", startDist);
        telemetry.addData("Regression", "slope=%.2f, intercept=%.1f",
                calibration.getSlope(), calibration.getIntercept());
        telemetry.addData("RPM at start", "%.0f", calibration.rpmForDistance(startDist));
        telemetry.update();

        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();
        if (isStopRequested()) { turretController.disable(); return; }

        reZeroHeadingAndTurret();
        flywheel.setShooterOn(true);

        // ╔═══════════════════════════════════════════════════════╗
        // ║                     MAIN LOOP                        ║
        // ╚═══════════════════════════════════════════════════════╝
        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            try { pinpoint.update(); } catch (Exception ignored) {}

            // FIX: Guard against (0,0) frames before localizer has a real fix
            if (follower != null) {
                follower.update();
                Pose rawPose = follower.getPose();
                if (rawPose != null) {
                    if (!followerPoseValid) {
                        double distFromOrigin = Math.hypot(rawPose.getX(), rawPose.getY());
                        if (distFromOrigin > 5.0) {
                            followerPoseValid = true;
                            currentPose = rawPose;
                        }
                        // else: keep using START_POSE as currentPose
                    } else {
                        currentPose = rawPose;
                    }
                }
            }

            // ── Touchpad reset (gp2) ──
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gamepad2TouchpadLast) {
                resetTurretEncoderAndReferences();
                driveController.stop();
            }
            gamepad2TouchpadLast = gp2Touch;

            // ── A-button reset (gp1) ──
            boolean aNow = gamepad1.a;
            if (aNow && !aPressedLast) {
                resetTurretEncoderAndReferences();
                driveController.stop();
            }
            aPressedLast = aNow;

            // ── Touchpad gp1: toggle far/close hood ──
            boolean touchNow = getTouchpad(gamepad1);
            if (touchNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
            }
            touchpadPressedLast = touchNow;

            // ── Drive ──
            driveController.setDrive(
                    -gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x, 1.0);

            // ── Shooter on/off (dpad down) ──
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            // ── Trim adjust (dpad left/right) ──
            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) flywheelVersatile.adjustTrim(-50.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) flywheelVersatile.adjustTrim(50.0);
            dpadRightLast = dpadRightNow;

            // ── Reset trim (dpad up) ──
            boolean dpadUpNow = gamepad1.dpad_up || gamepad2.dpad_up;
            if (dpadUpNow && !dpadUpLast) flywheelVersatile.resetTrim();
            dpadUpLast = dpadUpNow;

            // ── Back button: recalibrate regression live ──
            boolean backNow = gamepad1.back || gamepad2.back;
            if (backNow && !backPressedLast) flywheelVersatile.recalibrate();
            backPressedLast = backNow;

            // ── Gate toggle (B) ──
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) gateController.toggleGate();
            bPressedLast = bNow;

            // ── Intake sequence (Y) ──
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy())
                gateController.startIntakeSequence(nowMs);
            yPressedLast = yNow;

            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) clawController.trigger(nowMs);

            // ══════════════════════════════════════════════════════
            //  Flywheel: compute auto RPM from pose, apply to motor
            // ══════════════════════════════════════════════════════
            flywheel.handleLeftTrigger(
                    gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);

            double targetRpm = flywheelVersatile.getFinalTargetRPM(currentPose);
            flywheel.setTargetRPM(targetRpm);
            flywheel.update();

            // ── Hood auto-switch on PIDF mode change ──
            boolean pidfNow = flywheel.isUsingFarCoefficients();
            if (pidfNow != lastPidfMode) {
                hoodController.setRightPosition(pidfNow ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                lastPidfMode = pidfNow;
            }

            // ── Rumble at target ──
            if (flywheel.isAtTarget()) {
                try { gamepad1.rumble(200); } catch (Throwable ignored) {}
                try { gamepad2.rumble(200); } catch (Throwable ignored) {}
            }

            // ── Turret ──
            turretController.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);
            boolean manualTurret = false;
            double manualPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) {
                manualTurret = true; manualPower = 0.35;
            } else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) {
                manualTurret = true; manualPower = -0.35;
            }
            turretController.update(manualTurret, manualPower);

            // ── Manual intake ──
            if (!gateController.isBusy()) {
                if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                    intakeMotor.setPower(-1.0);
                } else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                    intakeMotor.setPower(1.0);
                } else {
                    intakeMotor.setPower(0.0);
                }
            }

            // ── Claw (X) ──
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) clawController.trigger(nowMs);
            xPressedLast = xNow;
            clawController.update(nowMs);

            // ── Hood nudge ──
            if (gamepad1.a) hoodController.nudgeLeftUp(nowMs);
            if (gamepad1.b) hoodController.nudgeLeftDown(nowMs);

            // ── Telemetry ──
            telemetry.addData("Flywheel",
                    "Cur: %.0f | Tgt: %.0f | Dist: %.1f",
                    flywheel.getCurrentRPM(), targetRpm,
                    flywheelVersatile.getLastDistance());
            telemetry.addData("Base RPM",   "%.0f", flywheelVersatile.getLastBaseRpm());
            telemetry.addData("Trim RPM",   "%.0f", flywheelVersatile.getTrimRpm());
            telemetry.addData("Pose", "%s %s",
                    followerPoseValid ? "✅" : "⏳",
                    currentPose != null
                            ? String.format("(%.1f, %.1f, %.1f°)",
                            currentPose.getX(), currentPose.getY(),
                            Math.toDegrees(currentPose.getHeading()))
                            : "N/A");
            telemetry.update();
        }

        turretController.disable();
    }

    // ═════════════════════════════════════════════════════════
    //  Helpers
    // ═════════════════════════════════════════════════════════

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
        turretController.captureReferences();
        turretController.resetPidState();
    }

    private void resetTurretEncoderAndReferences() {
        try { pinpoint.update(); } catch (Exception ignored) {}
        turretController.recenterAndResume(true);
    }
}