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

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.extras.TelemetryData;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.tracking.PinpointTurretAimer;

import java.util.List;

@TeleOp(name = "V3 Pinpoint Aim 🎯📍", group = "Linear OpMode")
public class PinpointAimTeleOp extends LinearOpMode {

    // Drive
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo, leftHoodServo, rightHoodServo, gateServo;
    private DigitalChannel turretLimitSwitch;

    // Controllers
    private PinpointTurretAimer turretAimer;
    private DriveController driveController;
    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;

    // Telemetry
    private TelemetryManager panelsTelemetry;
    private TelemetryData telemetryData;

    // Edge detection
    private boolean dpadDownLast, dpadLeftLast, dpadRightLast;
    private boolean xLast, bLast, yLast, aLast;
    private boolean touchpadLast, gp2TouchpadLast;

    private boolean isFarMode = false;
    private boolean lastPidfMode = false;

    // Constants
    private static final double RIGHT_HOOD_CLOSE = 0.16;
    private static final double RIGHT_HOOD_FAR = 0.26;
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.485;
    private static final long INTAKE_DURATION_MS = 1050;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 400;
    private static final double INTAKE_SEQ_POWER = 1.0;
    private static final double HOOD_MIN = 0.12;
    private static final double HOOD_MAX = 0.45;
    private static final double HOOD_LEFT_STEP = 0.025;
    private static final double HOOD_RIGHT_STEP = 0.01;
    private static final long HOOD_DEBOUNCE_MS = 120L;

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // ── Bulk read ──
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // ── Hardware ──
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRight");
        shooter    = hardwareMap.get(DcMotor.class, "shooter");
        shooter2   = hardwareMap.get(DcMotor.class, "shooter2");
        turret     = hardwareMap.get(DcMotor.class, "turret");
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

        // ── Directions ──
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        turret.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ── CRITICAL: Zero the turret encoder while it's physically straight forward ──
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── Pinpoint: the ONLY sensor we use for position + heading ──
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        // ── Turret aimer (pinpoint only — no Pedro Pathing heading) ──
        turretAimer = new PinpointTurretAimer(turret, pinpoint, telemetry);
        if (turretLimitSwitch != null) {
            turretAimer.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

        // ── Other subsystems ──
        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new FlywheelController(shooter, shooter2, telemetry, batterySensor);
        flywheel.setShooterOn(false);

        telemetryData = new TelemetryData(telemetry, flywheel);

        gateController = new GateController(
                gateServo, intakeMotor,
                led1Red, led1Green, led2Red, led2Green,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQ_POWER
        );

        clawController = new ClawController(clawServo, ClawController.CLAW_OPEN, ClawController.CLAW_CLOSED, ClawController.CLAW_CLOSE_MS);
        hoodController = new HoodController(
                leftHoodServo, rightHoodServo,
                HOOD_MIN, RIGHT_HOOD_CLOSE,
                HOOD_MIN, HOOD_MAX,
                HOOD_LEFT_STEP, HOOD_RIGHT_STEP,
                HOOD_DEBOUNCE_MS
        );

        gateController.setGateClosed(true);

        telemetry.addData("Status", "V3 Pinpoint Aim — ready");
        telemetry.addData("Goal", "(%.0f, %.0f)", PinpointTurretAimer.GOAL_X, PinpointTurretAimer.GOAL_Y);
        telemetry.addData("NOTE", "Turret must be straight forward at init!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) { turretAimer.disable(); return; }

        // Pinpoint is already zeroed from resetPosAndIMU().
        // Turret encoder is already zeroed from STOP_AND_RESET_ENCODER.
        // No "capture references" needed!
        flywheel.setShooterOn(true);

        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();
            telemetryData.updateLoopTime();

            // ── Update pinpoint (MUST be called every loop) ──
            try { pinpoint.update(); } catch (Exception ignored) {}

            // ════════════ RESETS ════════════

            // A button: zero turret encoder (must be straight forward!)
            boolean aNow = gamepad1.a;
            if (aNow && !aLast) {
                turretAimer.fullReset();
                driveController.stop();
            }
            aLast = aNow;

            // Gamepad2 touchpad: same reset
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gp2TouchpadLast) {
                turretAimer.fullReset();
                driveController.stop();
            }
            gp2TouchpadLast = gp2Touch;

            // Far/close toggle
            boolean touchpadNow = getTouchpad(gamepad1);
            if (touchpadNow && !touchpadLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
            }
            touchpadLast = touchpadNow;

            // ════════════ DRIVE ════════════
            driveController.setDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1.0);

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
            if (bNow && !bLast && !gateController.isBusy()) gateController.toggleGate();
            bLast = bNow;

            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yLast && !gateController.isBusy()) gateController.startIntakeSequence(nowMs);
            yLast = yNow;

            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) clawController.trigger(nowMs);

            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            boolean currentPidfMode = flywheel.isUsingFarCoefficients();
            if (currentPidfMode != lastPidfMode) {
                hoodController.setRightPosition(currentPidfMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                lastPidfMode = currentPidfMode;
            }

            if (flywheel.isAtTarget()) {
                try { gamepad1.rumble(200); } catch (Throwable ignored) {}
                try { gamepad2.rumble(200); } catch (Throwable ignored) {}
            }

            // ════════════ TURRET (pinpoint aim) ════════════
            turretAimer.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);

            boolean manualNow = false;
            double manualPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) {
                manualNow = true; manualPower = 0.35;
            } else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) {
                manualNow = true; manualPower = -0.35;
            }

            turretAimer.update(manualNow, manualPower);

            // ════════════ INTAKE ════════════
            if (!gateController.isBusy()) {
                if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                    intakeMotor.setPower(-1.0);
                } else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                    intakeMotor.setPower(1.0);
                } else {
                    intakeMotor.setPower(0.0);
                }
            }

            // ════════════ CLAW ════════════
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xLast) clawController.trigger(nowMs);
            xLast = xNow;
            clawController.update(nowMs);

            // ════════════ HOOD (gamepad2 only) ════════════
            if (gamepad2.a) hoodController.nudgeLeftUp(nowMs);
            if (gamepad2.b) hoodController.nudgeLeftDown(nowMs);

            // ════════════ TELEMETRY ════════════
            telemetry.addData("Aim.Error", "%.1f°", turretAimer.getErrorDeg());
            telemetry.addData("Aim.Dist", "%.1f in", turretAimer.getDistToGoal());
            telemetry.addData("Aim.Mode", turretAimer.isFreezeMode() ? "FREEZE 🔒" :
                    (turretAimer.isHomingMode() ? "HOMING 🔄" : "AIM 🎯"));
            telemetryData.update();
        }

        turretAimer.disable();
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