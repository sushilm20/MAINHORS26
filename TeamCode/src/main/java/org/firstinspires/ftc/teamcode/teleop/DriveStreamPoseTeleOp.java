package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tracking.CameraStreamManager;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.TagFamily;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "DriveStreamPoseTeleOp", group = "TeleOp")
public class DriveStreamPoseTeleOp extends OpMode {

    // Dashboard-configurable parameters
    @Sorter(sort = 1)
    public static double RANGE_SCALE = 2.7; // multiplicative correction

    @Sorter(sort = 2)
    public static double RANGE_BIAS = 0.0;  // additive correction (cm)

    @Sorter(sort = 3)
    public static int STREAM_FPS = 45;      // desired stream FPS

    @Sorter(sort = 4)
    public static double CAMERA_HEIGHT_CM = 30.0; // lens height from floor

    @Sorter(sort = 5)
    public static double TAG_HEIGHT_CM = 30.0;    // tag center height from floor

    @Sorter(sort = 6)
    public static int CAM_RES_WIDTH = 640;        // camera stream width

    @Sorter(sort = 7)
    public static int CAM_RES_HEIGHT = 480;       // camera stream height

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Pose currentPose = new Pose();

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final CameraStreamManager cameraStreamManager = new CameraStreamManager();
    private final ElapsedTime detectionTimer = new ElapsedTime();
    private boolean sawDetectionLastLoop = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("INIT", "Starting init");
        initFollower();
        initVision(hardwareMap);
        panelsTelemetry.debug("INIT", "Init complete");
        panelsTelemetry.update(telemetry);
    }

    private void initFollower() {
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));
            panelsTelemetry.debug("FOLLOWER", "Follower init");
        } catch (Exception e) {
            panelsTelemetry.debug("FOLLOWER", "Follower initialization fail");
            follower = null;
        }
    }

    private void initVision(HardwareMap hardwareMap) {
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
            portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam1")); // adjust name if needed
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

    /**
     * Expose VisionPortal for external inspection if needed.
     */
    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    @Override
    public void start() {
        panelsTelemetry.debug("START", "TeleOp start");
        if (follower != null) {
            follower.startTeleopDrive();
            panelsTelemetry.debug("FOLLOWER", "TeleOp drive started");
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();
            currentPose = follower.getPose();
            double axial = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;
            follower.setTeleOpDrive(axial, lateral, yaw, true);
        }

        panelsTelemetry.debug("POSE", currentPose != null
                ? String.format(Locale.US, "(%.1f, %.1f, %.1f°)", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()))
                : "N/A");

        addDetectionTelemetry();

        panelsTelemetry.update(telemetry);
    }

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
            double adjRange = correctedRange(d.ftcPose.range); // calibrated distance only
            double pitch = d.ftcPose.pitch;   // up/down
            double roll  = d.ftcPose.roll;    // alignment
            double yaw   = d.ftcPose.yaw;     // side/side

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

    /** Compute horizontal distance accounting for camera and tag heights. */
    private double horizontalDistance(double rawRange) {
        double dz = TAG_HEIGHT_CM - CAMERA_HEIGHT_CM;
        double squared = rawRange * rawRange - dz * dz;
        return squared > 0 ? Math.sqrt(squared) : 0.0;
    }

    @Override
    public void stop() {
        panelsTelemetry.debug("STOP", "Stopping");
        if (visionPortal != null) {
            visionPortal.close();
            panelsTelemetry.debug("STOP", "VisionPortal closed");
        }
        cameraStreamManager.stopPanelsStream();
        panelsTelemetry.debug("STOP", "Stream stopped");
        panelsTelemetry.update(telemetry);
    }
}