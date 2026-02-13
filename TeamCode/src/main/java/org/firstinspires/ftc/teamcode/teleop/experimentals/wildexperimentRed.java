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
import org.firstinspires.ftc.teamcode.subsystems.DistanceTracker;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.tracking.CalibrationPoints;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

@TeleOp(name = "HORS RED 🔴", group = "Linear OpMode")
public class wildexperimentRed extends LinearOpMode {

    private static final boolean IS_RED_ALLIANCE = true;
    private static final Pose START_POSE = CalibrationPoints.getStartPose(true);

    // Hardware
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo, leftHoodServo, rightHoodServo, gateServo;
    private DigitalChannel turretLimitSwitch;

    // Controllers
    private TurretController turretController;
    private DriveController driveController;
    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;
    private DistanceTracker distanceTracker;

    private TelemetryManager panelsTelemetry;

    // Button state
    private boolean dpadDownLast, dpadLeftLast, dpadRightLast;
    private boolean xPressedLast, bPressedLast, yPressedLast;
    private boolean touchpadPressedLast, gamepad2TouchpadLast, aPressedLast;

    // Mode toggles
    private boolean autoModeEnabled = true;
    private double rpmTrim = 0.0;
    private double hoodTrim = 0.0;

    // IMU
    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;

    // Follower
    private Follower follower;

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // ===== HARDWARE =====
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
        } catch (Exception e) { turretLimitSwitch = null; }

        LED led1Red = getLedSafe("led_1_red");
        LED led1Green = getLedSafe("led_1_green");
        LED led2Red = getLedSafe("led_2_red");
        LED led2Green = getLedSafe("led_2_green");

        VoltageSensor batterySensor = null;
        try { batterySensor = hardwareMap.voltageSensor.iterator().next(); } catch (Exception ignored) {}

        // Motor directions
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

        // IMU
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        try { imu = hardwareMap.get(BNO055IMU.class, "imu"); } catch (Exception e) { imu = null; }
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            if (pinpoint != null) pinpoint.resetPosAndIMU();
        } catch (Exception e) { pinpoint = null; }

        if (imu != null) { try { imu.initialize(imuParams); } catch (Exception ignored) {} }

        // Follower
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(START_POSE);
        } catch (Exception e) {
            telemetry.addData("Error", "Follower init failed!");
            follower = null;
        }

        // ===== CONTROLLERS =====
        turretController = new TurretController(turret, imu, pinpoint, telemetry);
        if (turretLimitSwitch != null) {
            turretController.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new FlywheelController(shooter, shooter2, telemetry, batterySensor);
        flywheel.setShooterOn(false);

        gateController = new GateController(gateServo, intakeMotor, led1Red, led1Green, led2Red, led2Green,
                CalibrationPoints.GATE_OPEN, CalibrationPoints.GATE_CLOSED,
                CalibrationPoints.INTAKE_DURATION_MS, CalibrationPoints.CLAW_TRIGGER_BEFORE_END_MS,
                CalibrationPoints.INTAKE_SEQUENCE_POWER);

        clawController = new ClawController(clawServo, CalibrationPoints.CLAW_OPEN,
                CalibrationPoints.CLAW_CLOSED, CalibrationPoints.CLAW_CLOSE_MS);

        hoodController = new HoodController(leftHoodServo, rightHoodServo,
                CalibrationPoints.HOOD_MIN, CalibrationPoints.HOOD_MIN,
                CalibrationPoints.HOOD_MIN, CalibrationPoints.HOOD_MAX,
                CalibrationPoints.HOOD_LEFT_STEP, CalibrationPoints.HOOD_RIGHT_STEP,
                CalibrationPoints.HOOD_DEBOUNCE_MS);

        // Distance tracker - handles red alliance mirroring
        distanceTracker = new DistanceTracker(IS_RED_ALLIANCE);

        // Set initial values
        flywheel.setTargetRPM(distanceTracker.getRpm());
        hoodController.setRightPosition(distanceTracker.getHood());
        gateController.setGateClosed(true);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", "RED 🔴");
        telemetry.addData("Start Pose", "(%.1f, %.1f)", START_POSE.getX(), START_POSE.getY());
        telemetry.addData("Start Distance", "%.1f", distanceTracker.getDistance());
        telemetry.addData("Start RPM", "%.0f", distanceTracker.getRpm());
        telemetry.addData("Start Hood", "%.3f", distanceTracker.getHood());
        telemetry.update();

        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();

        if (isStopRequested()) { turretController.disable(); return; }

        if (pinpoint != null) { try { pinpoint.update(); } catch (Exception ignored) {} }
        turretController.captureReferences();
        turretController.resetPidState();
        flywheel.setShooterOn(true);

        // ==================== MAIN LOOP ====================
        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            if (pinpoint != null) { try { pinpoint.update(); } catch (Exception ignored) {} }

            if (follower != null) {
                follower.update();
            }
            distanceTracker.update(follower);

            // Reset
            if (getTouchpad(gamepad2) && !gamepad2TouchpadLast) { resetAll(); }
            gamepad2TouchpadLast = getTouchpad(gamepad2);

            if (gamepad1.a && !aPressedLast) { resetAll(); }
            aPressedLast = gamepad1.a;

            // Toggle auto mode
            boolean touchpadNow = getTouchpad(gamepad1) || gamepad2.right_bumper;
            if (touchpadNow && !touchpadPressedLast) {
                autoModeEnabled = !autoModeEnabled;
            }
            touchpadPressedLast = touchpadNow;

            // Drive
            driveController.setDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1.0);

            // Flywheel controls
            if ((gamepad1.dpad_down || gamepad2.dpad_down) && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = gamepad1.dpad_down || gamepad2.dpad_down;

            if ((gamepad1.dpad_left || gamepad2.dpad_left) && !dpadLeftLast) rpmTrim -= 50.0;
            dpadLeftLast = gamepad1.dpad_left || gamepad2.dpad_left;

            if ((gamepad1.dpad_right || gamepad2.dpad_right) && !dpadRightLast) rpmTrim += 50.0;
            dpadRightLast = gamepad1.dpad_right || gamepad2.dpad_right;

            // Set RPM
            if (autoModeEnabled) {
                double targetRpm = distanceTracker.getRpm() + rpmTrim;
                targetRpm = Math.max(CalibrationPoints.RPM_MIN, Math.min(CalibrationPoints.RPM_MAX, targetRpm));
                flywheel.setTargetRPM(targetRpm);
            }

            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, gamepad1.back || gamepad2.back);

            // Set Hood
            if (autoModeEnabled) {
                double targetHood = distanceTracker.getHood() + hoodTrim;
                targetHood = Math.max(CalibrationPoints.HOOD_MIN, Math.min(CalibrationPoints.HOOD_MAX, targetHood));
                hoodController.setRightPosition(targetHood);
            }

            if (gamepad2.left_stick_y < -0.5) hoodTrim += CalibrationPoints.HOOD_TRIM_STEP;
            else if (gamepad2.left_stick_y > 0.5) hoodTrim -= CalibrationPoints.HOOD_TRIM_STEP;

            // Gate
            if ((gamepad1.b || gamepad2.b) && !bPressedLast && !gateController.isBusy()) gateController.toggleGate();
            bPressedLast = gamepad1.b || gamepad2.b;

            // Intake sequence
            if ((gamepad1.y || gamepad2.y) && !yPressedLast && !gateController.isBusy()) gateController.startIntakeSequence(nowMs);
            yPressedLast = gamepad1.y || gamepad2.y;

            if (gateController.update(nowMs)) clawController.trigger(nowMs);

            // Claw
            if ((gamepad1.x || gamepad2.x) && !xPressedLast) clawController.trigger(nowMs);
            xPressedLast = gamepad1.x || gamepad2.x;
            clawController.update(nowMs);

            // Intake manual
            if (!gateController.isBusy()) {
                if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) intakeMotor.setPower(-1.0);
                else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) intakeMotor.setPower(1.0);
                else intakeMotor.setPower(0.0);
            }

            // Turret
            turretController.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);
            boolean manualTurret = false;
            double turretPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) { manualTurret = true; turretPower = 0.35; }
            else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) { manualTurret = true; turretPower = -0.35; }
            turretController.update(manualTurret, turretPower);

            // Rumble
            if (flywheel.isAtTarget()) {
                try { gamepad1.rumble(200); } catch (Throwable ignored) {}
                try { gamepad2.rumble(200); } catch (Throwable ignored) {}
            }

            // Telemetry
            telemetry.addData("Alliance", "RED 🔴");
            telemetry.addData("Mode", autoModeEnabled ? "AUTO 🎯" : "MANUAL");
            telemetry.addLine();
            telemetry.addData("Position", "(%.1f, %.1f)", distanceTracker.getX(), distanceTracker.getY());
            telemetry.addData("Distance", "%.1f inches", distanceTracker.getDistance());
            telemetry.addData("Pose Valid", distanceTracker.wasLastPoseValid() ? "YES ✓" : "NO ✗ " + distanceTracker.getRejectReason());
            telemetry.addLine();
            telemetry.addData("RPM", "%.0f / %.0f (trim: %.0f)", flywheel.getCurrentRPM(), flywheel.getTargetRPM(), rpmTrim);
            telemetry.addData("Hood", "%.3f (trim: %.3f)", hoodController.getRightPos(), hoodTrim);

            if (follower != null) {
                Pose raw = follower.getPose();
                if (raw != null) {
                    telemetry.addData("Raw Follower", "(%.1f, %.1f)", raw.getX(), raw.getY());
                }
            }

            panelsTelemetry.debug("X", String.format("%.1f", distanceTracker.getX()));
            panelsTelemetry.debug("Y", String.format("%.1f", distanceTracker.getY()));
            panelsTelemetry.debug("Dist", String.format("%.1f", distanceTracker.getDistance()));
            panelsTelemetry.debug("RPM", String.format("%.0f", flywheel.getTargetRPM()));
            panelsTelemetry.debug("Hood", String.format("%.3f", hoodController.getRightPos()));
            panelsTelemetry.debug("Valid", distanceTracker.wasLastPoseValid() ? "YES" : "NO");

            telemetry.update();
            panelsTelemetry.update(telemetry);
        }

        turretController.disable();
    }

    private void resetAll() {
        if (follower != null) follower.setStartingPose(START_POSE);
        distanceTracker.reset();
        rpmTrim = 0.0;
        hoodTrim = 0.0;
        if (pinpoint != null) { try { pinpoint.update(); } catch (Exception ignored) {} }
        turretController.recenterAndResume(true);
        driveController.stop();
        gamepad1.rumble(300);
    }

    private boolean getTouchpad(com.qualcomm.robotcore.hardware.Gamepad gp) {
        try { return gp.touchpad; } catch (Throwable t) { return gp.left_stick_button && gp.right_stick_button; }
    }

    private LED getLedSafe(String name) {
        try { return hardwareMap.get(LED.class, name); } catch (Exception ignored) { return null; }
    }
}