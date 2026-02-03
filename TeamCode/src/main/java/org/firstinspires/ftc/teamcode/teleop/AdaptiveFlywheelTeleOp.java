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
import org.firstinspires.ftc.teamcode.subsystems.BallisticsCalculator;
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

    // ==================== VISION/STREAM CONFIGURATION ====================
    @Sorter(sort = 0) public static double RANGE_SCALE = 1.0;
    @Sorter(sort = 1) public static double RANGE_BIAS = 0.0;
    @Sorter(sort = 2) public static int STREAM_FPS = 30;
    @Sorter(sort = 3) public static int CAM_RES_WIDTH = 640;
    @Sorter(sort = 4) public static int CAM_RES_HEIGHT = 480;

    // ==================== ADAPTIVE MODE SETTINGS ====================
    @Sorter(sort = 5) public static boolean ADAPTIVE_MODE_ENABLED = true;
    @Sorter(sort = 6) public static int TARGET_APRILTAG_ID = -1;
    @Sorter(sort = 7) public static boolean ADAPTIVE_HOOD_ENABLED = true;
    @Sorter(sort = 8) public static boolean TURRET_LEAD_ENABLED = true;

    // ==================== GOAL POSITION (for bearing calculation) ====================
    @Sorter(sort = 9) public static double GOAL_X = 72.0;
    @Sorter(sort = 10) public static double GOAL_Y = 72.0;

    // ==================== HARDWARE ====================
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, shooter2, turret, intakeMotor;
    private Servo clawServo;
    private Servo leftHoodServo, rightHoodServo;
    private Servo gateServo;
    private DigitalChannel turretLimitSwitch;

    // ==================== CONTROLLERS ====================
    private TurretController turretController;
    private DriveController driveController;
    private FlywheelController flywheel;
    private GateController gateController;
    private ClawController clawController;
    private HoodController hoodController;
    private AdaptiveRPM adaptiveRPM;

    // ==================== VISION ====================
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private CameraStreamManager cameraStreamManager;
    private final ElapsedTime detectionTimer = new ElapsedTime();

    // ==================== TELEMETRY ====================
    private TelemetryManager panelsTelemetry;

    // ==================== POSE TRACKING ====================
    private Follower follower;
    private Pose currentPose = new Pose();
    private Pose lastPose = new Pose();
    private long lastPoseTimeMs = 0;
    private double robotVx = 0, robotVy = 0;

    // ==================== IMUs ====================
    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;

    // ==================== TOGGLE STATE ====================
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
    private boolean optionsPressedLast = false;
    private boolean sharePressedLast = false;
    private boolean guidePressedLast = false;

    private boolean isFarMode = false;
    private boolean adaptiveModeActive = true;
    private boolean adaptiveHoodActive = true;
    private boolean turretLeadActive = true;

    private double manualHoodOffset = 0.0;
    private double manualTurretLeadOffset = 0.0;

    // ==================== CONSTANTS ====================
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

    private static final double MANUAL_HOOD_OFFSET_STEP = 0.005;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        initializeHardware();
        initializeVision();
        initializeControllers();

        adaptiveRPM = new AdaptiveRPM();
        adaptiveModeActive = ADAPTIVE_MODE_ENABLED;
        adaptiveHoodActive = ADAPTIVE_HOOD_ENABLED;
        turretLeadActive = TURRET_LEAD_ENABLED;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", AdaptiveRPM.USE_PHYSICS_MODEL ? "PHYSICS" : "REGRESSION");
        telemetry.addData("Calibration", AdaptiveRPM.getRegressionInfo());
        telemetry.update();

        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();

        if (isStopRequested()) {
            cleanup();
            return;
        }

        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        reZeroHeadingAndTurret(imuParams);
        flywheel.setShooterOn(true);
        lastPoseTimeMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            if (isStopRequested()) break;

            long nowMs = System.currentTimeMillis();

            if (pinpoint != null) {
                try { pinpoint.update(); } catch (Exception ignored) {}
            }

            if (follower != null) {
                follower.update();
                currentPose = follower.getPose();
                calculateRobotVelocity(nowMs);
            }

            double bearingToGoal = calculateBearingToGoal();
            double detectedDistance = processVisionAndGetDistance();

            if (adaptiveModeActive) {
                if (turretLeadActive) {
                    adaptiveRPM.setRobotVelocity(robotVx, robotVy, bearingToGoal);
                } else {
                    adaptiveRPM.setRobotVelocity(0, 0, 0);
                }

                double targetRpm = adaptiveRPM.update(detectedDistance, nowMs);
                flywheel.setTargetRpm(targetRpm);

                if (adaptiveHoodActive) {
                    double targetHood = adaptiveRPM.getTargetHoodPosition();
                    targetHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, targetHood + manualHoodOffset));
                    rightHoodServo.setPosition(targetHood);
                }

                isFarMode = targetRpm >= FlywheelController.RPM_SWITCH_THRESHOLD;
            }

            handleResetControls(nowMs, imuParams);
            handleModeToggles(nowMs);
            handleDriveControls();
            handleFlywheelControls(nowMs);
            handleGateAndIntakeControls(nowMs);
            handleClawControls(nowMs);
            handleHoodControls(nowMs);
            handleTurretControls(nowMs);

            if (flywheel.isAtTarget()) {
                try { gamepad1.rumble(200); } catch (Throwable ignored) {}
                try { gamepad2.rumble(200); } catch (Throwable ignored) {}
            }

            updateTelemetry(detectedDistance, bearingToGoal);
        }

        cleanup();
    }

    // ==================== VELOCITY & BEARING CALCULATIONS ====================

    private void calculateRobotVelocity(long nowMs) {
        if (currentPose == null || lastPose == null) {
            robotVx = 0;
            robotVy = 0;
            return;
        }

        long dtMs = nowMs - lastPoseTimeMs;
        if (dtMs <= 0) {
            return;
        }

        double dt = dtMs / 1000.0;

        double dx = currentPose.getX() - lastPose.getX();
        double dy = currentPose.getY() - lastPose.getY();

        robotVx = dx / dt;
        robotVy = dy / dt;

        lastPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        lastPoseTimeMs = nowMs;
    }

    private double calculateBearingToGoal() {
        if (currentPose == null) {
            return 0;
        }

        double dx = GOAL_X - currentPose.getX();
        double dy = GOAL_Y - currentPose.getY();

        double fieldBearing = Math.atan2(dy, dx);
        double robotBearing = fieldBearing - currentPose.getHeading();

        while (robotBearing > Math.PI) robotBearing -= 2 * Math.PI;
        while (robotBearing < -Math.PI) robotBearing += 2 * Math.PI;

        return Math.toDegrees(robotBearing);
    }

    // ==================== INITIALIZATION ====================

    private void initializeHardware() {
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

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        turret.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        } catch (IllegalArgumentException e) {
            portalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }

        visionPortal = portalBuilder.build();
        cameraStreamManager.startPanelsStream(STREAM_FPS);
        detectionTimer.reset();
    }

    private void initializeControllers() {
        LED led1Red = getLedSafe("led_1_red");
        LED led1Green = getLedSafe("led_1_green");
        LED led2Red = getLedSafe("led_2_red");
        LED led2Green = getLedSafe("led_2_green");

        VoltageSensor batterySensor = null;
        try {
            batterySensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {}

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

        gateController.setGateClosed(true);
    }

    // ==================== VISION ====================

    private double processVisionAndGetDistance() {
        if (aprilTagProcessor == null) {
            cameraStreamManager.setOverlayText("Vision not ready");
            return -1.0;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        cameraStreamManager.updateAprilTagDetections(detections);

        if (detections.isEmpty()) {
            String mode = AdaptiveRPM.USE_PHYSICS_MODEL ? "PHY" : "REG";
            String overlayText = String.format(Locale.US, "No Tag | %s | RPM:%.0f | Hood:%.3f",
                    mode, flywheel.getTargetRPM(), adaptiveRPM.getLastOutputHood());
            cameraStreamManager.setOverlayText(overlayText);
            return -1.0;
        }

        AprilTagDetection targetDetection = null;
        for (AprilTagDetection detection : detections) {
            if (TARGET_APRILTAG_ID == -1 || detection.id == TARGET_APRILTAG_ID) {
                targetDetection = detection;
                break;
            }
        }

        if (targetDetection == null || targetDetection.ftcPose == null) {
            cameraStreamManager.setOverlayText("Target not found");
            return -1.0;
        }

        double rawDistance = targetDetection.ftcPose.range;
        double calibratedDistance = (rawDistance * RANGE_SCALE) + RANGE_BIAS;

        String mode = AdaptiveRPM.USE_PHYSICS_MODEL ? "PHY" : "REG";
        String overlayText = String.format(Locale.US,
                "ID:%d | Dist:%.1f\" | RPM:%.0f | Hood:%.3f | %s",
                targetDetection.id, calibratedDistance,
                flywheel.getTargetRPM(), adaptiveRPM.getLastOutputHood(), mode);
        cameraStreamManager.setOverlayText(overlayText);

        detectionTimer.reset();
        return calibratedDistance;
    }

    // ==================== CONTROL HANDLERS ====================

    private void handleResetControls(long nowMs, BNO055IMU.Parameters imuParams) {
        boolean gp2Touch = getTouchpad(gamepad2);
        if (gp2Touch && !gamepad2TouchpadLast) {
            resetTurretEncoderAndReferences(imuParams);
            driveController.stop();
            adaptiveRPM.reset();
            manualHoodOffset = 0.0;
            manualTurretLeadOffset = 0.0;
        }
        gamepad2TouchpadLast = gp2Touch;

        boolean aNow = gamepad1.a;
        if (aNow && !aPressedLast) {
            resetTurretEncoderAndReferences(imuParams);
            driveController.stop();
            adaptiveRPM.reset();
            manualHoodOffset = 0.0;
            manualTurretLeadOffset = 0.0;
        }
        aPressedLast = aNow;
    }

    private void handleModeToggles(long nowMs) {
        // Toggle adaptive RPM
        boolean optionsNow = gamepad1.options || gamepad2.options;
        if (optionsNow && !optionsPressedLast) {
            adaptiveModeActive = !adaptiveModeActive;
            if (!adaptiveModeActive) {
                flywheel.setCloseMode();
                isFarMode = false;
            } else {
                adaptiveRPM.reset();
                manualHoodOffset = 0.0;
            }
        }
        optionsPressedLast = optionsNow;

        // Toggle adaptive hood
        boolean shareNow = gamepad1.share || gamepad2.share;
        if (shareNow && !sharePressedLast) {
            adaptiveHoodActive = !adaptiveHoodActive;
        }
        sharePressedLast = shareNow;

        // Toggle physics/regression mode
        boolean guideNow = gamepad1.guide || gamepad2.guide;
        if (guideNow && !guidePressedLast) {
            AdaptiveRPM.USE_PHYSICS_MODEL = !AdaptiveRPM.USE_PHYSICS_MODEL;
            BallisticsCalculator.USE_PHYSICS_MODEL = AdaptiveRPM.USE_PHYSICS_MODEL;
            adaptiveRPM.reset();
        }
        guidePressedLast = guideNow;

        // Manual far/close toggle
        boolean touchpadNow = getTouchpad(gamepad1);
        if (touchpadNow && !touchpadPressedLast) {
            if (!adaptiveModeActive) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                if (!adaptiveHoodActive) {
                    rightHoodServo.setPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                }
            } else {
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
        boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
        if (dpadDownNow && !dpadDownLast) {
            flywheel.toggleShooterOn();
        }
        dpadDownLast = dpadDownNow;

        boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
        if (dpadLeftNow && !dpadLeftLast) {
            if (adaptiveModeActive) {
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

        boolean calibPressed = gamepad1.back || gamepad2.back;
        flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
        flywheel.update(nowMs, calibPressed);
    }

    private void handleGateAndIntakeControls(long nowMs) {
        boolean bNow = gamepad1.b || gamepad2.b;
        if (bNow && !bPressedLast && !gateController.isBusy()) {
            gateController.toggleGate();
        }
        bPressedLast = bNow;

        boolean yNow = gamepad1.y || gamepad2.y;
        if (yNow && !yPressedLast && !gateController.isBusy()) {
            gateController.startIntakeSequence(nowMs);
        }
        yPressedLast = yNow;

        boolean shouldTriggerClaw = gateController.update(nowMs);
        if (shouldTriggerClaw) {
            clawController.trigger(nowMs);
        }

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

        if (adaptiveHoodActive && adaptiveModeActive) {
            if (gamepad2.right_stick_y < -0.2) {
                manualHoodOffset += MANUAL_HOOD_OFFSET_STEP;
                manualHoodOffset = Math.min(manualHoodOffset, 0.1);
            } else if (gamepad2.right_stick_y > 0.2) {
                manualHoodOffset -= MANUAL_HOOD_OFFSET_STEP;
                manualHoodOffset = Math.max(manualHoodOffset, -0.1);
            }
        } else {
            if (gamepad2.right_stick_y < -0.2) {
                hoodController.nudgeRightUp(nowMs);
            } else if (gamepad2.right_stick_y > 0.2) {
                hoodController.nudgeRightDown(nowMs);
            }
        }
    }

    private void handleTurretControls(long nowMs) {
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

    private void updateTelemetry(double detectedDistance, double bearingToGoal) {
        String modeStr = AdaptiveRPM.USE_PHYSICS_MODEL ? "PHYSICS" : "REGRESSION";
        telemetry.addData("Model", modeStr);
        telemetry.addData("Adaptive", "%s RPM | %s Hood | %s Lead",
                adaptiveModeActive ? "ON" : "OFF",
                adaptiveHoodActive ? "ON" : "OFF",
                turretLeadActive ? "ON" : "OFF");

        telemetry.addData("Flywheel", "Cur:%.0f | Tgt:%.0f RPM",
                flywheel.getCurrentRPM(), flywheel.getTargetRPM());
        telemetry.addData("PIDF", flywheel.isUsingFarCoefficients() ? "FAR" : "CLOSE");

        double currentHood = adaptiveRPM.getLastOutputHood() + manualHoodOffset;
        telemetry.addData("Hood", "Pos:%.3f | Offset:%+.3f", currentHood, manualHoodOffset);

        if (adaptiveModeActive) {
            telemetry.addData("Adaptive", adaptiveRPM.getStatusString());
            if (detectedDistance > 0) {
                telemetry.addData("Distance", "%.1f inches", detectedDistance);
            }
            if (turretLeadActive && adaptiveRPM.hasValidDetection()) {
                telemetry.addData("Turret Lead", "%.1f°", adaptiveRPM.getTurretLeadAngleDeg());
            }
        }

        telemetry.addData("Velocity", "Vx:%.1f Vy:%.1f in/s", robotVx, robotVy);
        telemetry.addData("Bearing", "%.1f°", bearingToGoal);

        telemetry.addData("Pose", currentPose != null
                ? String.format("(%.1f, %.1f, %.1f°)",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()))
                : "N/A");

        telemetry.addLine();
        telemetry.addData("Controls", "OPT:Adaptive | SHARE:Hood | GUIDE:Physics");

        panelsTelemetry.debug("FLYWHEEL", String.format("RPM:%.0f/%.0f | Hood:%.3f",
                flywheel.getCurrentRPM(), flywheel.getTargetRPM(), currentHood));
        panelsTelemetry.debug("MODE", String.format("%s | %s",
                adaptiveModeActive ? "AUTO" : "MANUAL", modeStr));

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