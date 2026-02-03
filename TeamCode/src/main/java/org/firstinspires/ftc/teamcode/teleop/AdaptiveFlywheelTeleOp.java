package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AdaptiveRPM;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.GateController;
import org.firstinspires.ftc.teamcode.subsystems.HoodController;
import org.firstinspires.ftc.teamcode.tracking.CameraStreamManager;
import org.firstinspires.ftc.teamcode.tracking.TurretController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "HORS Adaptive Flywheel 🎯", group = "TeleOp")
public class AdaptiveFlywheelTeleOp extends LinearOpMode {

    // --- Vision/Stream configuration ---
    @Sorter(sort = 0) public static double RANGE_SCALE = 1.0;       // Distance calibration multiplier
    @Sorter(sort = 1) public static double RANGE_BIAS = 0.0;        // Distance calibration offset (inches)
    @Sorter(sort = 2) public static int STREAM_FPS = 30;            // Camera stream FPS
    @Sorter(sort = 3) public static int CAM_RES_WIDTH = 640;        // Camera resolution width
    @Sorter(sort = 4) public static int CAM_RES_HEIGHT = 480;       // Camera resolution height

    // --- Adaptive mode toggle ---
    @Sorter(sort = 5) public static boolean ADAPTIVE_MODE_ENABLED = true;  // Enable/disable adaptive RPM

    // --- Target AprilTag ID (set to -1 to use any detected tag) ---
    @Sorter(sort = 6) public static int TARGET_APRILTAG_ID = -1;

    // --- Hardware ---
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;
    private DigitalChannel turretLimitSwitch;

    // --- Controllers ---
    private TurretController turretController;
    private DriveController driveController;
    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;
    private AdaptiveRPM adaptiveRPM;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private CameraStreamManager cameraStreamManager;
    private final ElapsedTime detectionTimer = new ElapsedTime();

    // --- Telemetry ---
    private TelemetryManager panelsTelemetry;

    // --- Pose tracking ---
    private Follower follower;
    private Pose currentPose = new Pose();

    // --- IMUs ---
    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;

    // --- Toggle state tracking ---
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
    private boolean optionsPressedLast = false;  // For toggling adaptive mode

    private boolean isFarMode = false;
    private boolean lastPidfMode = false;
    private boolean adaptiveModeActive = true;  // Runtime toggle

    // --- Constants (same as wildexperiment) ---
    private static final double RIGHT_HOOD_CLOSE = 0.16;
    private static final double RIGHT_HOOD_FAR = 0.24;

    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.467;
    private static final long INTAKE_DURATION_MS = 1050;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 400;
    private static final double INTAKE_SEQUENCE_POWER = 1.0;

    private static final double CLAW_OPEN = 0.63;
    private static final double CLAW_CLOSED = 0.2;
    private static final long CLAW_CLOSE_MS = 500L;

    private static final double HOOD_MIN = 0.12;
    private static final double HOOD_MAX = 0.45;
    private static final double HOOD_LEFT_STEP = 0.025;
    private static final double HOOD_RIGHT_STEP = 0.01;
    private static final long HOOD_DEBOUNCE_MS = 120L;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize all hardware
        initializeHardware();

        // Initialize vision system
        initializeVision();

        // Initialize controllers
        initializeControllers();

        // Create AdaptiveRPM controller
        adaptiveRPM = new AdaptiveRPM();
        adaptiveModeActive = ADAPTIVE_MODE_ENABLED;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Adaptive Mode", adaptiveModeActive ? "ENABLED" : "DISABLED");
        telemetry.addData("RPM Regression", String.format("%.2f * dist + %.1f",
                AdaptiveRPM.getSlope(), AdaptiveRPM.getIntercept()));
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
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        reZeroHeadingAndTurret(imuParams);
        flywheel.setShooterOn(true);

        // Main loop
        while (opModeIsActive()) {
            if (isStopRequested()) break;

            long nowMs = System.currentTimeMillis();

            // Update pinpoint IMU
            if (pinpoint != null) {
                try { pinpoint.update(); } catch (Exception ignored) {}
            }

            // Update follower for pose
            if (follower != null) {
                follower.update();
                currentPose = follower.getPose();
            }

            // Process vision and get distance
            double detectedDistance = processVisionAndGetDistance();

            // Update adaptive RPM if enabled
            if (adaptiveModeActive) {
                double targetRpm = adaptiveRPM.update(detectedDistance, nowMs);
                flywheel.setTargetRpm(targetRpm);

                // Update far mode state based on RPM threshold
                boolean shouldBeFarMode = targetRpm >= FlywheelController.RPM_SWITCH_THRESHOLD;
                if (shouldBeFarMode != isFarMode) {
                    isFarMode = shouldBeFarMode;
                    // Hood will auto-adjust via PIDF mode change detection below
                }
            }

            // Handle input controls
            handleResetControls(nowMs, imuParams);
            handleModeToggles(nowMs);
            handleDriveControls();
            handleFlywheelControls(nowMs);
            handleGateAndIntakeControls(nowMs);
            handleClawControls(nowMs);
            handleHoodControls(nowMs);
            handleTurretControls();

            // Check PIDF mode change and update hood
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

            // Update telemetry
            updateTelemetry(detectedDistance);
        }

        cleanup();
    }

    // ==================== INITIALIZATION ====================

    private void initializeHardware() {
        // Drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        // Shooter/turret/intake motors
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        turret = hardwareMap.get(DcMotor.class, "turret");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // Servos
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // Turret limit switch
        try {
            turretLimitSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
            turretLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            turretLimitSwitch = null;
        }

        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        turret.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Motor modes
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU initialization
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

        // Follower initialization
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));
        } catch (Exception e) {
            follower = null;
        }
    }

    private void initializeVision() {
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
            panelsTelemetry.debug("VISION", "webcam1 not found, using phone camera");
            portalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }

        visionPortal = portalBuilder.build();
        cameraStreamManager.startPanelsStream(STREAM_FPS);
        detectionTimer.reset();
    }

    private void initializeControllers() {
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

        // Create controllers
        turretController = new TurretController(turret, imu, pinpoint, telemetry);
        if (turretLimitSwitch != null) {
            turretController.setEncoderResetTrigger(() -> !turretLimitSwitch.getState());
        }

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

        clawController = new ClawController(clawServo, CLAW_OPEN, CLAW_CLOSED, CLAW_CLOSE_MS);

        hoodController = new HoodController(
                leftHoodServo, rightHoodServo,
                HOOD_MIN, RIGHT_HOOD_CLOSE,
                HOOD_MIN, HOOD_MAX,
                HOOD_LEFT_STEP, HOOD_RIGHT_STEP,
                HOOD_DEBOUNCE_MS
        );

        // Initial positions
        gateController.setGateClosed(true);
    }

    // ==================== VISION PROCESSING ====================

    /**
     * Process AprilTag detections and return distance in inches.
     * Returns -1 if no valid detection.
     */
    private double processVisionAndGetDistance() {
        if (aprilTagProcessor == null) {
            cameraStreamManager.setOverlayText("Vision not ready");
            return -1.0;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        // Update camera stream with detections for visual overlay
        cameraStreamManager.updateAprilTagDetections(detections);

        if (detections.isEmpty()) {
            String overlayText = String.format(Locale.US, "No Tag | Mode: %s | RPM: %.0f",
                    adaptiveModeActive ? "ADAPTIVE" : "MANUAL",
                    flywheel.getTargetRPM());
            cameraStreamManager.setOverlayText(overlayText);
            return -1.0;
        }

        // Find the target tag or use first detection
        AprilTagDetection targetDetection = null;
        for (AprilTagDetection detection : detections) {
            if (TARGET_APRILTAG_ID == -1 || detection.id == TARGET_APRILTAG_ID) {
                targetDetection = detection;
                break;
            }
        }

        if (targetDetection == null) {
            cameraStreamManager.setOverlayText("Target tag not found");
            return -1.0;
        }

        if (targetDetection.ftcPose == null) {
            cameraStreamManager.setOverlayText(String.format("Tag %d - No pose", targetDetection.id));
            return -1.0;
        }

        // Get calibrated distance
        double rawDistance = targetDetection.ftcPose.range;
        double calibratedDistance = (rawDistance * RANGE_SCALE) + RANGE_BIAS;

        // Build overlay text
        String overlayText = String.format(Locale.US,
                "ID:%d | Dist:%.1f in | RPM:%.0f | %s",
                targetDetection.id,
                calibratedDistance,
                flywheel.getTargetRPM(),
                adaptiveModeActive ? "ADAPTIVE" : "MANUAL");
        cameraStreamManager.setOverlayText(overlayText);

        detectionTimer.reset();
        return calibratedDistance;
    }

    // ==================== CONTROL HANDLERS ====================

    private void handleResetControls(long nowMs, BNO055IMU.Parameters imuParams) {
        // Gamepad2 touchpad reset
        boolean gp2Touch = getTouchpad(gamepad2);
        if (gp2Touch && !gamepad2TouchpadLast) {
            resetTurretEncoderAndReferences(imuParams);
            driveController.stop();
            adaptiveRPM.reset();
        }
        gamepad2TouchpadLast = gp2Touch;

        // Gamepad1 A button reset
        boolean aNow = gamepad1.a;
        if (aNow && !aPressedLast) {
            resetTurretEncoderAndReferences(imuParams);
            driveController.stop();
            adaptiveRPM.reset();
        }
        aPressedLast = aNow;
    }

    private void handleModeToggles(long nowMs) {
        // Toggle adaptive mode with OPTIONS/BACK button
        boolean optionsNow = gamepad1.options || gamepad2.options;
        if (optionsNow && !optionsPressedLast) {
            adaptiveModeActive = !adaptiveModeActive;
            if (!adaptiveModeActive) {
                // When switching to manual, set to close RPM
                flywheel.setCloseMode();
                isFarMode = false;
            } else {
                // Reset adaptive controller when re-enabling
                adaptiveRPM.reset();
            }
        }
        optionsPressedLast = optionsNow;

        // Manual far/close toggle on gamepad1 touchpad (only in manual mode)
        boolean touchpadNow = getTouchpad(gamepad1);
        if (touchpadNow && !touchpadPressedLast) {
            if (!adaptiveModeActive) {
                // Manual mode: toggle far/close
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
            } else {
                // Adaptive mode: touchpad toggles adaptive on/off
                adaptiveModeActive = false;
                flywheel.setCloseMode();
                isFarMode = false;
            }
        }
        touchpadPressedLast = touchpadNow;
    }

    private void handleDriveControls() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        driveController.setDrive(axial, lateral, yaw, 1.0);
    }

    private void handleFlywheelControls(long nowMs) {
        // Toggle shooter on/off
        boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
        if (dpadDownNow && !dpadDownLast) {
            flywheel.toggleShooterOn();
        }
        dpadDownLast = dpadDownNow;

        // Manual RPM adjustment (only works in manual mode or as override)
        boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
        if (dpadLeftNow && !dpadLeftLast) {
            if (adaptiveModeActive) {
                // In adaptive mode, dpad left/right temporarily disables adaptive
                adaptiveModeActive = false;
            }
            flywheel.adjustTargetRPM(-50.0);
        }
        dpadLeftLast = dpadLeftNow;

        boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
        if (dpadRightNow && !dpadRightLast) {
            if (adaptiveModeActive) {
                adaptiveModeActive = false;
            }
            flywheel.adjustTargetRPM(50.0);
        }
        dpadRightLast = dpadRightNow;

        // Left trigger handling
        boolean calibPressed = gamepad1.back || gamepad2.back;
        flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
        flywheel.update(nowMs, calibPressed);
    }

    private void handleGateAndIntakeControls(long nowMs) {
        // Gate toggle (B)
        boolean bNow = gamepad1.b || gamepad2.b;
        if (bNow && !bPressedLast && !gateController.isBusy()) {
            gateController.toggleGate();
        }
        bPressedLast = bNow;

        // Intake sequence (Y)
        boolean yNow = gamepad1.y || gamepad2.y;
        if (yNow && !yPressedLast && !gateController.isBusy()) {
            gateController.startIntakeSequence(nowMs);
        }
        yPressedLast = yNow;

        // Gate update + claw trigger
        boolean shouldTriggerClaw = gateController.update(nowMs);
        if (shouldTriggerClaw) {
            clawController.trigger(nowMs);
        }

        // Manual intake (only if gate not busy)
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
    }

    private void handleClawControls(long nowMs) {
        boolean xNow = gamepad1.x || gamepad2.x;
        if (xNow && !xPressedLast) {
            clawController.trigger(nowMs);
        }
        xPressedLast = xNow;
        clawController.update(nowMs);
    }

    private void handleHoodControls(long nowMs) {
        if (gamepad1.a) hoodController.nudgeLeftUp(nowMs);
        if (gamepad1.b) hoodController.nudgeLeftDown(nowMs);
        if (gamepad2.right_stick_y < -0.2) hoodController.nudgeRightUp(nowMs);
        else if (gamepad2.right_stick_y > 0.2) hoodController.nudgeRightDown(nowMs);
    }

    private void handleTurretControls() {
        turretController.commandHomingSweep(gamepad1.dpad_up);

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
    }

    // ==================== TELEMETRY ====================

    private void updateTelemetry(double detectedDistance) {
        // Flywheel info
        telemetry.addData("Mode", adaptiveModeActive ? "ADAPTIVE 🎯" : "MANUAL");
        telemetry.addData("Flywheel", "Current: %.0f | Target: %.0f RPM",
                flywheel.getCurrentRPM(), flywheel.getTargetRPM());
        telemetry.addData("PIDF Mode", flywheel.isUsingFarCoefficients() ? "FAR" : "CLOSE");

        // Adaptive info
        if (adaptiveModeActive) {
            telemetry.addData("Adaptive", adaptiveRPM.getStatusString());
            if (detectedDistance > 0) {
                telemetry.addData("Distance", "%.1f inches", detectedDistance);
            }
        }

        // Pose info
        telemetry.addData("Pose", currentPose != null
                ? String.format("(%.1f, %.1f, %.1f°)",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()))
                : "N/A");

        // Controls reminder
        telemetry.addLine();
        telemetry.addData("Controls", "OPTIONS: Toggle Adaptive | DPAD L/R: Manual RPM");

        panelsTelemetry.debug("FLYWHEEL", String.format("RPM: %.0f/%.0f | %s",
                flywheel.getCurrentRPM(), flywheel.getTargetRPM(),
                adaptiveModeActive ? "ADAPTIVE" : "MANUAL"));
        panelsTelemetry.debug("DISTANCE", detectedDistance > 0 ?
                String.format("%.1f in", detectedDistance) : "No tag");

        telemetry.update();
        panelsTelemetry.update(telemetry);
    }

    // ==================== UTILITIES ====================

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
        if (visionPortal != null) {
            visionPortal.close();
        }
        cameraStreamManager.stopPanelsStream();
    }
}