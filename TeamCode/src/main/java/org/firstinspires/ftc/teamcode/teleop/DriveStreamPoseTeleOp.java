package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import androidx.annotation.Nullable;

import com.bylazar.camerastream.PanelsCameraStream;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.TagFamily;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "DriveStreamPoseTeleOp", group = "TeleOp")
public class DriveStreamPoseTeleOp extends OpMode {

    private Follower follower;
    private Pose currentPose = new Pose();

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final StreamProcessor streamProcessor = new StreamProcessor();
    private final ElapsedTime detectionTimer = new ElapsedTime();

    @Override
    public void init() {
        initFollower();
        initVision(hardwareMap);
    }

    private void initFollower() {
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));
        } catch (Exception e) {
            telemetry.addData("Error", "Follower initialization failed!");
            follower = null;
        }
    }

    private void initVision(HardwareMap hardwareMap) {
        aprilTagProcessor = new Builder()
                .setDrawAxis(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setTagFamily(TagFamily.TAG_36h11)
                .build();

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .addProcessor(streamProcessor);

        try {
            portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } catch (Exception ignored) {
            portalBuilder.setCamera(VisionPortal.CameraMode.WEBCAM);
        }

        visionPortal = portalBuilder.build();
        PanelsCameraStream.startStream(streamProcessor);
        detectionTimer.reset();
    }

    @Override
    public void start() {
        if (follower != null) {
            follower.startTeleopDrive();
        }
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

        telemetry.addData("Pose", currentPose != null
                ? String.format("(%.1f, %.1f, %.1f°)", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()))
                : "N/A");

        addDetectionTelemetry();
        telemetry.update();
    }

    private void addDetectionTelemetry() {
        if (aprilTagProcessor == null) {
            telemetry.addData("AprilTag", "Processor not initialized");
            return;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        telemetry.addData("Tags Seen", detections.size());

        if (detections.isEmpty()) {
            telemetry.addData("Tag", "None");
            telemetry.addData("Last Detection (s ago)", "%.1f", detectionTimer.seconds());
            return;
        }

        AprilTagDetection detection = detections.get(0);
        telemetry.addData("Tag ID", detection.id);
        if (detection.ftcPose != null) {
            telemetry.addData("Range (cm)", "%.1f", detection.ftcPose.range);
            telemetry.addData("Bearing (deg)", "%.1f", detection.ftcPose.bearing);
            telemetry.addData("Yaw (deg)", "%.1f", detection.ftcPose.yaw);
        }
        telemetry.addData("Last Detection (s ago)", "%.1f", detectionTimer.seconds());
        detectionTimer.reset();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
        PanelsCameraStream.stopStream();
    }

    /**
     * Minimal processor that copies frames to a bitmap for Panels streaming.
     */
    private static class StreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, @Nullable CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = lastFrame.get();
            if (b.getWidth() != frame.width() || b.getHeight() != frame.height()) {
                b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            }
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // Panels handles drawing; nothing needed here.
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
}
