package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.tracking.TurretController;

import java.util.List;

@Disabled
@TeleOp(name="A HORS OFFICIAL ⭐", group="Linear OpMode")
public class OfficialHORS extends LinearOpMode {

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

    // Drive scaling
    private static final double NORMAL_DRIVE_SCALE = 1.0;
    private static final double SLOW_DRIVE_SCALE = 0.45;

    // Rumble cooldown
    private static final long RUMBLE_COOLDOWN_MS = 1200;
    private long lastAtTargetRumbleMs = 0L;

    // Pinpoint IMU only
    private GoBildaPinpointDriver pinpoint;

    // Pose tracking
    private Follower follower;
    private Pose currentPose = new Pose();

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Bulk read
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

        try {
            turretLimitSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
            turretLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            turretLimitSwitch = null;
        }

        LED led1Red = getLedSafe("led_1_red");
        LED led1Green = getLedSafe("led_1_green");
        LED led2Red = getLedSafe("led_2_red");
        LED led2Green = getLedSafe("led_2_green");

        VoltageSensor batterySensor = null;
        try { batterySensor = hardwareMap.voltageSensor.iterator().next(); }
        catch (Exception ignored) {}

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

        // Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));
        } catch (Exception e) {
            telemetry.addData("Error", "Follower initialization failed!");
            follower = null;
        }

        turretController = new TurretController(turret, null, pinpoint, telemetry);
        if (turretLimitSwitch != null) {
            turretController.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

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

        gateController.setGateClosed(true);
        telemetry.addData("Status", "Initialized (mode = CLOSE, shooter OFF)");
        telemetry.addData("Drive", "Input shaping ON | SlowMode: LT");
        telemetry.update();

        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();

        if (isStopRequested()) {
            turretController.disable();
            return;
        }

        reZeroHeadingAndTurret();
        flywheel.setShooterOn(true);

        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            telemetryData.updateLoopTime();

            try { pinpoint.update(); } catch (Exception ignored) {}
            if (follower != null) {
                follower.update();
                currentPose = follower.getPose();
            }

            // Reset controls
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gamepad2TouchpadLast) {
                resetTurretEncoderAndReferences();
                driveController.stop();
            }
            gamepad2TouchpadLast = gp2Touch;

            boolean aNow = gamepad1.a;
            if (aNow && !aPressedLast) {
                resetTurretEncoderAndReferences();
                driveController.stop();
            }
            aPressedLast = aNow;

            // Far/close
            boolean touchpadNow = getTouchpad(gamepad1);
            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
            }
            touchpadPressedLast = touchpadNow;

            // Drive (slow mode on left trigger)
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double scale = (gamepad1.left_trigger > 0.25) ? SLOW_DRIVE_SCALE : NORMAL_DRIVE_SCALE;
            driveController.setDrive(axial, lateral, yaw, scale);

            // Flywheel DPAD
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) flywheel.adjustTargetRPM(-50.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) flywheel.adjustTargetRPM(50.0);
            dpadRightLast = dpadRightNow;

            // Gate
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) gateController.toggleGate();
            bPressedLast = bNow;

            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy()) gateController.startIntakeSequence(nowMs);
            yPressedLast = yNow;

            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) clawController.trigger(nowMs);

            // Flywheel update
            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            boolean currentPidfMode = flywheel.isUsingFarCoefficients();
            if (currentPidfMode != lastPidfMode) {
                hoodController.setRightPosition(currentPidfMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                lastPidfMode = currentPidfMode;
            }

            // Rumble only on edge + cooldown
            if (flywheel.justReachedTarget() && (nowMs - lastAtTargetRumbleMs >= RUMBLE_COOLDOWN_MS)) {
                final int RUMBLE_MS = 220;
                try { gamepad1.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
                lastAtTargetRumbleMs = nowMs;
            }

            // Turret
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
                if (leftTriggerNow) intakeMotor.setPower(-1.0);
                else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) intakeMotor.setPower(1.0);
                else intakeMotor.setPower(0.0);
            }

            // Claw
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) clawController.trigger(nowMs);
            xPressedLast = xNow;
            clawController.update(nowMs);

            // Hood nudges
            if (gamepad1.a) hoodController.nudgeLeftUp(nowMs);
            if (gamepad1.b) hoodController.nudgeLeftDown(nowMs);

            telemetryData.setPose(currentPose);
            telemetry.addData("DriveScale", "%.2f", scale);
            telemetryData.update();
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

    private void reZeroHeadingAndTurret() {
        try { pinpoint.update(); } catch (Exception ignored) {}
        turretController.captureReferences();
        turretController.resetPidState();
    }

    private void resetTurretEncoderAndReferences() {
        try { pinpoint.update(); } catch (Exception ignored) {}
        turretController.recenterAndResume(true);


        //should i lowk write my inspirational message as a comment here? maybe not, but here it is anyway:
        // "In the middle of difficulty lies opportunity." - Albert Einstein

        /*This method is intended to be called when the turret limit switch is triggered, indicating that the turret is in a known
        reference position. It resets the turret's encoder and updates the reference position for the PID controller.
         This allows the turret to maintain accurate positioning even if it gets knocked out of alignment during a match.

         I initially didn't intend to write a post season message but I'm doing it because of how much this club means to me.

         From the day I joined the club, I saw Subham, David, Jaden, Brandon, and the rest of the team working tirelessly
         on the robot, always pushing the boundaries of what we could achieve. At the start I initially didn't think much
         of the club than just for building my resume for college but I've come to realize this club is so much more than that.
         If i had to recall a special movement that will stay with me for the rest of my life, it would be the moment of centerstage
         where we qualified for WORLD. From then, I come to reali

         */
    }
}