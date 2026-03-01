package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.tracking.CameraTurretController;
import org.firstinspires.ftc.teamcode.tracking.CameraStreamManager;
import org.firstinspires.ftc.teamcode.tracking.TurretController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@Disabled
@TeleOp(name="Camera Tracking TeleOp 🎯", group="Linear OpMode")
public class CameraTracking extends LinearOpMode {

    // ===================== CAMERA SETTINGS =====================
    private static final int CAM_RES_WIDTH = 640;
    private static final int CAM_RES_HEIGHT = 480;
    private static final int STREAM_FPS = 30;

    // Target AprilTag ID to track (-1 for any tag)
    private static final int TARGET_TAG_ID = 22;

    // ===================== HARDWARE =====================
    // Drive motors
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    // Subsystem motors
    private DcMotor shooter, shooter2, turret, intakeMotor;
    // Servos
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;
    // Sensors
    private DigitalChannel turretLimitSwitch;

    // ===================== CONTROLLERS =====================
    private TurretController turretController;           // IMU-based heading compensation
    private CameraTurretController cameraTurretController; // AprilTag tracking
    private DriveController driveController;
    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;

    // ===================== VISION =====================
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private CameraStreamManager cameraStreamManager;

    // ===================== TELEMETRY =====================
    private TelemetryManager panelsTelemetry;

    // ===================== STATE TRACKING =====================
    // Button latches
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
    private boolean gamepad2RightBumperLast = false;
    private boolean backPressedLast = false;  // For tracking mode toggle

    // Mode flags
    private boolean isFarMode = false;
    private boolean lastPidfMode = false;

    // Turret tracking mode: false = IMU-based (TurretController), true = Camera-based (CameraTurretController)
    private boolean useCameraTracking = false;

    // ===================== CONSTANTS =====================
    // Hood presets
    private static final double RIGHT_HOOD_CLOSE = 0.16;
    private static final double RIGHT_HOOD_FAR = 0.24;

    // Gate/Intake
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.467;
    private static final long INTAKE_DURATION_MS = 1050;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 400;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;

    // Hood
    private static final double HOOD_MIN = 0.12;
    private static final double HOOD_MAX = 0.45;
    private static final double HOOD_LEFT_STEP = 0.025;
    private static final double HOOD_RIGHT_STEP = 0.01;
    private static final long HOOD_DEBOUNCE_MS = 120L;

    // IMUs
    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;

    // Pose tracking
    private Follower follower;
    private Pose currentPose = new Pose();

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // ==================== HARDWARE INIT ====================
        initHardware();

        // ==================== VISION INIT ====================
        initVision();

        // ==================== IMU INIT ====================
        BNO055IMU.Parameters imuParams = initIMU();

        // ==================== FOLLOWER INIT ====================
        initFollower();

        String imuUsed = (pinpoint != null) ? "pinpoint" : (imu != null) ? "imu (expansion hub)" : "none";

        // ==================== CONTROLLERS INIT ====================
        initControllers(imuParams);

        // Initial positions
        gateController.setGateClosed(true);

        telemetry.addData("Status", "Initialized (mode = CLOSE, shooter OFF)");
        telemetry.addData("Tracking Mode", "IMU-based (press BACK to toggle)");
        telemetry.addData("RPM Switch Threshold", "%.0f RPM", FlywheelController.RPM_SWITCH_THRESHOLD);
        telemetry.addData("Turret IMU", imuUsed);
        telemetry.addData("Vision", visionPortal != null ? "Ready" : "Failed");
        telemetry.update();

        // Prepare subsystems
        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();

        if (isStopRequested()) {
            cleanup();
            return;
        }

        // After start: capture current state
        reZeroHeadingAndTurret(imuParams);
        flywheel.setShooterOn(true);

        // Start camera stream
        if (cameraStreamManager != null) {
            cameraStreamManager.startPanelsStream(STREAM_FPS);
        }

        // ==================== MAIN LOOP ====================
        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            // Refresh sensors
            updateSensors();

            // Process inputs and update subsystems
            processResetInputs(imuParams, nowMs);
            processModeToggles(nowMs);
            processDriveInputs();
            processFlywheelInputs(nowMs);
            processGateAndIntakeInputs(nowMs);
            processTurretInputs(nowMs);
            processClawInputs(nowMs);
            processHoodInputs(nowMs);

            // Update vision overlay
            updateVisionOverlay();

            // Update telemetry
            updateTelemetry();
        }

        cleanup();
    }

    // ==================== INITIALIZATION METHODS ====================

    private void initHardware() {
        // Drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        // Subsystem motors
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        turret = hardwareMap.get(DcMotor.class, "turret");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Servos
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // Optional turret limit switch
        try {
            turretLimitSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
            turretLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            turretLimitSwitch = null;
        }

        // Motor directions & modes
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
    }

    private void initVision() {
        try {
            cameraStreamManager = new CameraStreamManager();

            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawTagOutline(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .build();

            VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                    .addProcessor(aprilTagProcessor)
                    .addProcessor(cameraStreamManager.getVisionProcessor())
                    .setCameraResolution(new Size(CAM_RES_WIDTH, CAM_RES_HEIGHT));

            try {
                portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam1"));
                panelsTelemetry.debug("VISION", "Using webcam1");
            } catch (IllegalArgumentException e) {
                portalBuilder.setCamera(BuiltinCameraDirection.BACK);
                panelsTelemetry.debug("VISION", "Using phone back camera");
            }

            visionPortal = portalBuilder.build();
            panelsTelemetry.debug("VISION", "Portal built successfully");
        } catch (Exception e) {
            panelsTelemetry.debug("VISION", "Init failed: " + e.getMessage());
            visionPortal = null;
            aprilTagProcessor = null;
            cameraStreamManager = null;
        }
    }

    private BNO055IMU.Parameters initIMU() {
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
            } catch (Exception ignored) {}
        }

        return imuParams;
    }

    private void initFollower() {
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20, 122, Math.toRadians(130)));
        } catch (Exception e) {
            telemetry.addData("Error", "Follower initialization failed!");
            follower = null;
        }
    }

    private void initControllers(BNO055IMU.Parameters imuParams) {
        // LEDs
        LED led1Red = getLedSafe("led_1_red");
        LED led1Green = getLedSafe("led_1_green");
        LED led2Red = getLedSafe("led_2_red");
        LED led2Green = getLedSafe("led_2_green");

        // Voltage sensor
        VoltageSensor batterySensor = null;
        try {
            batterySensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {}

        // IMU-based turret controller
        turretController = new TurretController(turret, imu, pinpoint, telemetry);
        if (turretLimitSwitch != null) {
            turretController.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

        // Camera-based turret controller
        cameraTurretController = new CameraTurretController(turret, aprilTagProcessor, telemetry);
        if (turretLimitSwitch != null) {
            cameraTurretController.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

        // Other controllers
        driveController = new DriveController(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        flywheel = new FlywheelController(shooter, shooter2, telemetry, batterySensor);
        flywheel.setShooterOn(false);

        gateController = new GateController(
                gateServo, intakeMotor,
                led1Red, led1Green, led2Red, led2Green,
                GATE_OPEN, GATE_CLOSED,
                INTAKE_DURATION_MS, CLAW_TRIGGER_BEFORE_END_MS,
                INTAKE_SEQUENCE_POWER
        );

        clawController = new ClawController(clawServo, ClawController.CLAW_OPEN, ClawController.CLAW_CLOSED, ClawController.CLAW_CLOSE_MS);
        hoodController = new HoodController(
                leftHoodServo, rightHoodServo,
                HOOD_MIN, RIGHT_HOOD_CLOSE,
                HOOD_MIN, HOOD_MAX,
                HOOD_LEFT_STEP, HOOD_RIGHT_STEP,
                HOOD_DEBOUNCE_MS
        );
    }

    // ==================== UPDATE METHODS ====================

    private void updateSensors() {
        if (pinpoint != null) {
            try {
                pinpoint.update();
            } catch (Exception ignored) {}
        }

        if (follower != null) {
            follower.update();
            currentPose = follower.getPose();
        }
    }

    private void processResetInputs(BNO055IMU.Parameters imuParams, long nowMs) {
        // Touchpad reset (gamepad2)
        boolean gp2Touch = getTouchpad(gamepad2);
        if (gp2Touch && !gamepad2TouchpadLast) {
            resetTurretEncoderAndReferences(imuParams);
            cameraTurretController.resetState();
            driveController.stop();
            telemetry.addData("Reset", "Turret encoder reset (gp2 touchpad)");
        }
        gamepad2TouchpadLast = gp2Touch;

        // A button reset (gamepad1)
        boolean aNow = gamepad1.a;
        if (aNow && !aPressedLast) {
            resetTurretEncoderAndReferences(imuParams);
            cameraTurretController.resetState();
            driveController.stop();
            telemetry.addData("Reset", "Turret encoder reset (gp1 A)");
        }
        aPressedLast = aNow;
    }

    private void processModeToggles(long nowMs) {
        // Far/close mode toggle (touchpad or gamepad2 right bumper)
        boolean touchpadNow = getTouchpad(gamepad1) || gamepad2.right_bumper;
        if (touchpadNow && !touchpadPressedLast) {
            isFarMode = !isFarMode;
            flywheel.setModeFar(isFarMode);
            hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
        }
        touchpadPressedLast = touchpadNow;

        // *** NEW: Tracking mode toggle (BACK button) ***
        // Press BACK on either gamepad to toggle between IMU and Camera tracking
        boolean backNow = gamepad1.back || gamepad2.back;
        if (backNow && !backPressedLast) {
            useCameraTracking = !useCameraTracking;

            // Reset state when switching modes
            if (useCameraTracking) {
                cameraTurretController.resetState();
                turretController.disable();
            } else {
                turretController.captureReferences();
                turretController.resetPidState();
                cameraTurretController.disable();
            }
        }
        backPressedLast = backNow;
    }

    private void processDriveInputs() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        driveController.setDrive(axial, lateral, yaw, 1.0);
    }

    private void processFlywheelInputs(long nowMs) {
        // DPAD toggles
        boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
        if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
        dpadDownLast = dpadDownNow;

        boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
        if (dpadLeftNow && !dpadLeftLast) flywheel.adjustTargetRPM(-50.0);
        dpadLeftLast = dpadLeftNow;

        boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
        if (dpadRightNow && !dpadRightLast) flywheel.adjustTargetRPM(50.0);
        dpadRightLast = dpadRightNow;

        // Flywheel update (NOTE: back button now used for tracking toggle, use guide for calib)
        boolean calibPressed = gamepad1.guide || gamepad2.guide;
        flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
        flywheel.update(nowMs, calibPressed);

        // Hood auto-adjust based on PIDF mode
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
    }

    private void processGateAndIntakeInputs(long nowMs) {
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

        // Manual intake (only if gate not busy)
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
    }

    private void processTurretInputs(long nowMs) {
        // Determine manual control state
        boolean manualNow = false;
        double manualPower = 0.0;

        if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) {
            manualNow = true;
            manualPower = 0.35;
        } else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) {
            manualNow = true;
            manualPower = -0.35;
        }

        // *** TRACKING MODE SELECTION ***
        if (useCameraTracking) {
            // Camera-based AprilTag tracking
            // Auto-align when holding right trigger (gamepad1 or gamepad2)
            boolean autoAlignTrigger = gamepad1.right_trigger > 0.3 || gamepad2.right_trigger > 0.3;

            // Set runtime for accurate timing
            cameraTurretController.setRuntime(getRuntime());

            // Update camera turret controller
            // If manual override is active, it takes priority
            if (manualNow) {
                cameraTurretController.update(false, manualPower, TARGET_TAG_ID);
            } else {
                cameraTurretController.update(autoAlignTrigger, 0.0, TARGET_TAG_ID);
            }
        } else {
            // IMU-based heading compensation (original TurretController)
            // Homing sweep on dpad_up or gamepad2 left_bumper
            turretController.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);
            turretController.update(manualNow, manualPower);
        }
    }

    private void processClawInputs(long nowMs) {
        boolean xNow = gamepad1.x || gamepad2.x;
        if (xNow && !xPressedLast) {
            clawController.trigger(nowMs);
        }
        xPressedLast = xNow;
        clawController.update(nowMs);
    }

    private void processHoodInputs(long nowMs) {
        if (gamepad1.a) hoodController.nudgeLeftUp(nowMs);
        if (gamepad1.b) hoodController.nudgeLeftDown(nowMs);
    }

    private void updateVisionOverlay() {
        if (cameraStreamManager == null || aprilTagProcessor == null) return;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        cameraStreamManager.updateAprilTagDetections(detections);

        // Build overlay text
        String overlayText;
        if (useCameraTracking) {
            if (cameraTurretController.isAligned()) {
                overlayText = String.format(Locale.US, "ALIGNED | Tag:%d | Bearing:%.1f°",
                        cameraTurretController.getLastDetectedTagId(),
                        cameraTurretController.getLastBearing());
            } else if (cameraTurretController.getLastDetectedTagId() >= 0) {
                overlayText = String.format(Locale.US, "TRACKING | Tag:%d | Error:%.1f° | Pwr:%.2f",
                        cameraTurretController.getLastDetectedTagId(),
                        cameraTurretController.getLastError(),
                        cameraTurretController.getLastAppliedPower());
            } else {
                overlayText = "CAM MODE | No Tag";
            }
        } else {
            overlayText = String.format(Locale.US, "IMU MODE | Turret:%d | Tags:%d",
                    turretController.getVirtualPosition(),
                    detections.size());
        }
        cameraStreamManager.setOverlayText(overlayText);
    }

    private void updateTelemetry() {
        // Flywheel
        telemetry.addData("Flywheel", "Current: %.0f rpm | Target: %.0f rpm",
                flywheel.getCurrentRPM(), flywheel.getTargetRPM());

        // Tracking mode
        telemetry.addData("Tracking Mode", useCameraTracking ? "CAMERA (AprilTag)" : "IMU (Heading)");

        if (useCameraTracking) {
            telemetry.addData("Camera Turret", "Tag:%d | Aligned:%s | Error:%.1f°",
                    cameraTurretController.getLastDetectedTagId(),
                    cameraTurretController.isAligned() ? "YES" : "NO",
                    cameraTurretController.getLastError());
        } else {
            telemetry.addData("IMU Turret", "Pos:%d | Error:%d",
                    turretController.getVirtualPosition(),
                    turretController.getLastErrorTicks());
        }

        // Pose
        telemetry.addData("Pose", currentPose != null
                ? String.format("(%.1f, %.1f, %.1f°)",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()))
                : "N/A");

        telemetry.update();
    }

    // ==================== HELPER METHODS ====================

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
            try { pinpoint.update(); } catch (Exception ignored) {}
        }
        turretController.captureReferences();
        turretController.resetPidState();
    }

    private void resetTurretEncoderAndReferences(BNO055IMU.Parameters imuParams) {
        if (pinpoint != null) {
            try { pinpoint.update(); } catch (Exception ignored) {}
        }
        turretController.recenterAndResume(true);
    }

    private void cleanup() {
        turretController.disable();
        cameraTurretController.disable();

        if (cameraStreamManager != null) {
            cameraStreamManager.stopPanelsStream();
        }
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}