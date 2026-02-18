package org.firstinspires.ftc.teamcode.teleop.experimentals;

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
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.TagFamily;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name="Virtual Slime Out 🎮📷", group="Linear OpMode")
public class FunCameraTeleop extends LinearOpMode {

    // ========== Camera Stream & Vision Parameters (from DriveStreamPoseTeleOp) ==========
    @Sorter(sort = 1)
    public static double RANGE_SCALE = 2.7; // multiplicative correction

    @Sorter(sort = 2)
    public static double RANGE_BIAS = 0.0;  // additive correction (cm)

    @Sorter(sort = 3)
    public static int STREAM_FPS = 60;      // desired stream FPS

    @Sorter(sort = 4)
    public static double CAMERA_HEIGHT_CM = 30.0; // lens height from floor

    @Sorter(sort = 5)
    public static double TAG_HEIGHT_CM = 30.0;    // tag center height from floor

    @Sorter(sort = 6)
    public static int CAM_RES_WIDTH = 640;        // camera stream width

    @Sorter(sort = 7)
    public static int CAM_RES_HEIGHT = 480;       // camera stream height

    // ========== Drive + subsystems (from OfficialHORS) ==========
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

    // ========== Vision & Camera Stream ==========
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final CameraStreamManager cameraStreamManager = new CameraStreamManager();
    private final ElapsedTime detectionTimer = new ElapsedTime();
    private boolean sawDetectionLastLoop = false;

    // ========== Telemetry ==========
    private TelemetryManager panelsTelemetry;

    // ========== Toggles / state (from OfficialHORS) ==========
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

    private boolean isFarMode = false;
    private boolean lastPidfMode = false;

    // ========== Constants (from OfficialHORS) ==========
    private static final double RIGHT_HOOD_CLOSE = 0.16;
    private static final double RIGHT_HOOD_FAR = 0.24;

    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.467;
    private static final long INTAKE_DURATION_MS = 1050;
    private static final long CLAW_TRIGGER_BEFORE_END_MS = 400;
    private static final double INTKE_SEQUENCE_POWER = 1.0;

    private static final double CLAW_OPEN = 0.63;
    private static final double CLAW_CLOSED = 0.2;
    private static final long CLAW_CLOSE_MS = 500L;

    private static final double HOOD_MIN = 0.12;
    private static final double HOOD_MAX = 0.45;
    private static final double HOOD_LEFT_STEP = 0.025;
    private static final double HOOD_RIGHT_STEP = 0.01;
    private static final long HOOD_DEBOUNCE_MS = 120L;

    // ========== IMUs & Pose tracking ==========
    private BNO055IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private Follower follower;
    private Pose currentPose = new Pose();

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("INIT", "Starting VirtualSlimeOut init");

        // ========== Hardware map (from OfficialHORS) ==========
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

        // Optional: turret limit switch
        try {
            turretLimitSwitch = hardwareMap.get(DigitalChannel.class, "turret_limit");
            turretLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            turretLimitSwitch = null;
        }

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

        // ========== IMU init ==========
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

        // ========== Follower init ==========
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20, 122, Math.toRadians(130)));
            panelsTelemetry.debug("FOLLOWER", "Follower init");
        } catch (Exception e) {
            telemetry.addData("Error", "Follower initialization failed!");
            panelsTelemetry.debug("FOLLOWER", "Follower initialization fail");
            follower = null;
        }

        String imuUsed = (pinpoint != null) ? "pinpoint" : (imu != null) ? "imu (expansion hub)" : "none";

        // ========== Vision & Camera Stream init (from DriveStreamPoseTeleOp) ==========
        initVision();

        // ========== Create controllers ==========
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
                INTKE_SEQUENCE_POWER
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
        telemetry.addData("Status", "VirtualSlimeOut Initialized (camera stream ON, shooter OFF)");
        telemetry.addData("RPM Switch Threshold", "%.0f RPM", FlywheelController.RPM_SWITCH_THRESHOLD);
        telemetry.addData("\nTurret IMU", imuUsed);
        telemetry.addData("Camera Stream", "Active @ " + STREAM_FPS + " fps");
        telemetry.update();

        // Prepare subsystems
        turretController.captureReferences();
        turretController.resetPidState();

        waitForStart();

        if (isStopRequested()) {
            turretController.disable();
            stopVision();
            return;
        }

        // After start: re-zero
        reZeroHeadingAndTurret(imuParams);
        flywheel.setShooterOn(true);
        panelsTelemetry.debug("START", "VirtualSlimeOut started");

        while (opModeIsActive()) {
            if (isStopRequested()) break;
            long nowMs = System.currentTimeMillis();

            // ========== Update pinpoint ==========
            if (pinpoint != null) {
                try { pinpoint.update(); } catch (Exception ignored) {}
            }

            // ========== Update follower & pose ==========
            if (follower != null) {
                follower.update();
                currentPose = follower.getPose();
            }

            // ========== Touchpad reset (gamepad2) ==========
            boolean gp2Touch = getTouchpad(gamepad2);
            if (gp2Touch && !gamepad2TouchpadLast) {
                resetTurretEncoderAndReferences(imuParams);
                driveController.stop();
                telemetry.addData("Reset", "Turret encoder reset & reference captured (gp2 touchpad)");
                telemetry.update();
            }
            gamepad2TouchpadLast = gp2Touch;

            // ========== A button reset (gamepad1) ==========
            boolean aNow = gamepad1.a;
            if (aNow && !aPressedLast) {
                resetTurretEncoderAndReferences(imuParams);
                driveController.stop();
                telemetry.addData("Reset", "Turret encoder reset & reference captured (gp1 A)");
                telemetry.update();
            }
            aPressedLast = aNow;

            // ========== Far/close toggle ==========
            boolean touchpadNow = getTouchpad(gamepad1) || gamepad2.right_bumper;
            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                flywheel.setModeFar(isFarMode);
                hoodController.setRightPosition(isFarMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
            }
            touchpadPressedLast = touchpadNow;

            // ========== Drive ==========
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 1.0);

            // ========== Flywheel toggles (DPAD) ==========
            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) flywheel.toggleShooterOn();
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) flywheel.adjustTargetRPM(-50.0);
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) flywheel.adjustTargetRPM(50.0);
            dpadRightLast = dpadRightNow;

            // ========== Gate manual toggle (B) ==========
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bPressedLast && !gateController.isBusy()) {
                gateController.toggleGate();
            }
            bPressedLast = bNow;

            // ========== Intake auto sequence (Y) ==========
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast && !gateController.isBusy()) {
                gateController.startIntakeSequence(nowMs);
            }
            yPressedLast = yNow;

            // ========== Gate sequence update + claw auto trigger ==========
            boolean shouldTriggerClaw = gateController.update(nowMs);
            if (shouldTriggerClaw) {
                clawController.trigger(nowMs);
            }

            // ========== Flywheel update ==========
            boolean calibPressed = gamepad1.back || gamepad2.back;
            flywheel.handleLeftTrigger(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1);
            flywheel.update(nowMs, calibPressed);

            // ========== Check if PIDF mode changed ==========
            boolean currentPidfMode = flywheel.isUsingFarCoefficients();
            if (currentPidfMode != lastPidfMode) {
                hoodController.setRightPosition(currentPidfMode ? RIGHT_HOOD_FAR : RIGHT_HOOD_CLOSE);
                lastPidfMode = currentPidfMode;
            }

            // ========== Rumble when at target ==========
            if (flywheel.isAtTarget()) {
                final int RUMBLE_MS = 200;
                try { gamepad1.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble(RUMBLE_MS); } catch (Throwable ignored) {}
            }

            // ========== Turret control ==========
            turretController.commandHomingSweep(gamepad1.dpad_up || gamepad2.left_bumper);

            boolean manualNow = false;
            double manualPower = 0.0;

            if (gamepad1.right_bumper || gamepad2.right_stick_x > 0.2) {
                manualNow = true; manualPower = 0.35;
            } else if (gamepad1.left_bumper || gamepad2.right_stick_x < -0.2) {
                manualNow = true; manualPower = -0.35;
            }

            turretController.update(manualNow, manualPower);

            // ========== Intake manual ==========
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

            // ========== Claw manual (X) ==========
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawController.trigger(nowMs);
            }
            xPressedLast = xNow;
            clawController.update(nowMs);

            // ========== Hood adjustments ==========
            if (gamepad1.a) hoodController.nudgeLeftUp(nowMs);
            if (gamepad1.b) hoodController.nudgeLeftDown(nowMs);

            // ========== AprilTag detection telemetry (from DriveStreamPoseTeleOp) ==========
            addDetectionTelemetry();

            // ========== Combined Telemetry ==========
            telemetry.addData("Flywheel", "Current: %.0f rpm | Target: %.0f rpm",
                    flywheel.getCurrentRPM(), flywheel.getTargetRPM());

            telemetry.addData("Pose", currentPose != null
                    ? String.format(Locale.US, "(%.1f, %.1f, %.1f°)",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()))
                    : "N/A");

            panelsTelemetry.update(telemetry);
        }

        // Cleanup
        turretController.disable();
        stopVision();
    }

    // ========== Vision Init (from DriveStreamPoseTeleOp) ==========
    private void initVision() {
        panelsTelemetry.debug("VISION", "Building AprilTag processor and portal");
        aprilTagProcessor = new Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setTagFamily(TagFamily.TAG_36h11)
                .build();

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .addProcessor(cameraStreamManager.getVisionProcessor())
                .setCameraResolution(new Size(CAM_RES_WIDTH, CAM_RES_HEIGHT));

        try {
            portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam1"));
            panelsTelemetry.debug("VISION", "Using webcam1");
        } catch (IllegalArgumentException e) {
            panelsTelemetry.debug("VISION", "webcam1 not found, using phone back camera");
            portalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }

        visionPortal = portalBuilder.build();
        cameraStreamManager.startPanelsStream(STREAM_FPS);
        detectionTimer.reset();
        panelsTelemetry.debug("VISION", "Portal built, stream started @ " + STREAM_FPS + " fps");
    }

    // ========== AprilTag Detection Telemetry (from DriveStreamPoseTeleOp) ==========
    private void addDetectionTelemetry() {
        if (aprilTagProcessor == null) {
            panelsTelemetry.debug("APRILTAG", "Processor not initialized");
            return;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections.isEmpty()) {
            panelsTelemetry.debug("APRILTAG", String.format(Locale.US,
                    "ID: None | Distance (cm): -- | Pitch: -- | Roll: -- | Yaw: -- | Last (s): %.1f",
                    detectionTimer.seconds()));
            sawDetectionLastLoop = false;
            cameraStreamManager.setOverlayText("No Tag");
            return;
        }

        if (!sawDetectionLastLoop) detectionTimer.reset();
        sawDetectionLastLoop = true;

        AprilTagDetection d = detections.get(0);
        if (d.ftcPose != null) {
            double adjRange = correctedRange(d.ftcPose.range);
            double pitch = d.ftcPose.pitch;
            double roll  = d.ftcPose.roll;
            double yaw   = d.ftcPose.yaw;

            panelsTelemetry.debug("APRILTAG", String.format(Locale.US,
                    "ID: %d | Distance (cm): %.1f | Pitch: %.1f | Roll: %.1f | Yaw: %.1f | Last (s): %.1f",
                    d.id, adjRange, pitch, roll, yaw, detectionTimer.seconds()));

            String overlay = String.format(Locale.US,
                    "ID:%d  Dist:%.1fcm  Pitch:%.1f  Roll:%.1f  Yaw:%.1f",
                    d.id, adjRange, pitch, roll, yaw);
            cameraStreamManager.setOverlayText(overlay);
        } else {
            panelsTelemetry.debug("APRILTAG", String.format(Locale.US,
                    "ID:%d | Distance (cm): -- | Pitch: -- | Roll: -- | Yaw: -- | Last (s): %.1f",
                    d.id, detectionTimer.seconds()));
            cameraStreamManager.setOverlayText("Tag seen, no pose");
        }
    }

    private double correctedRange(double raw) {
        return raw * RANGE_SCALE + RANGE_BIAS;
    }

    // ========== Vision Cleanup ==========
    private void stopVision() {
        panelsTelemetry.debug("STOP", "Stopping vision");
        if (visionPortal != null) {
            visionPortal.close();
            panelsTelemetry.debug("STOP", "VisionPortal closed");
        }
        cameraStreamManager.stopPanelsStream();
        panelsTelemetry.debug("STOP", "Stream stopped");
    }

    // ========== Helper Methods (from OfficialHORS) ==========
    private boolean getTouchpad(com.qualcomm.robotcore.hardware.Gamepad gp) {
        try { return gp.touchpad; }
        catch (Throwable t) { return gp.left_stick_button && gp.right_stick_button; }
    }

    private LED getLedSafe(String name) {
        try { return hardwareMap.get(LED.class, name); }
        catch (Exception ignored) { return null; }
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

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }
}