package org.firstinspires.ftc.teamcode.tracking;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

import androidx.annotation.Nullable;

import com.bylazar.camerastream.PanelsCameraStream;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/** Shared stream/overlay helper for Panels streaming with AprilTag visualization. */
public class CameraStreamManager {
    private final StreamProcessor streamProcessor = new StreamProcessor();

    public VisionProcessor getVisionProcessor() {
        return streamProcessor;
    }

    public CameraStreamSource getCameraStreamSource() {
        return streamProcessor;
    }

    public void startPanelsStream(int fps) {
        PanelsCameraStream.INSTANCE.startStream(streamProcessor, fps);
    }

    public void stopPanelsStream() {
        PanelsCameraStream.INSTANCE.stopStream();
    }

    public void setOverlayText(String text) {
        streamProcessor.setOverlayText(text);
    }

    /** Call this each frame with the latest AprilTag detections */
    public void updateAprilTagDetections(List<AprilTagDetection> detections) {
        streamProcessor.updateDetections(detections);
    }

    private static class StreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(null);
        private final AtomicReference<String> overlayText = new AtomicReference<>("");
        private final AtomicReference<List<AprilTagDetection>> currentDetections =
                new AtomicReference<>(new ArrayList<>());

        // Paints for drawing
        private final Paint boxPaint = new Paint();
        private final Paint textPaint = new Paint();
        private final Paint textBgPaint = new Paint();
        private final Paint cornerPaint = new Paint();
        private final Paint headerPaint = new Paint();
        private final Paint headerBgPaint = new Paint();

        private int frameWidth = 640;
        private int frameHeight = 480;

        StreamProcessor() {
            // Box outline paint (green)
            boxPaint.setColor(Color.GREEN);
            boxPaint.setStyle(Paint.Style.STROKE);
            boxPaint.setStrokeWidth(4f);
            boxPaint.setAntiAlias(true);

            // Corner dots paint (cyan)
            cornerPaint.setColor(Color.CYAN);
            cornerPaint.setStyle(Paint.Style.FILL);
            cornerPaint.setAntiAlias(true);

            // Tag info text paint (white)
            textPaint.setColor(Color.WHITE);
            textPaint.setTextSize(32f);
            textPaint.setAntiAlias(true);
            textPaint.setFakeBoldText(true);

            // Tag info background paint (semi-transparent dark)
            textBgPaint.setColor(Color.argb(180, 0, 0, 0));
            textBgPaint.setStyle(Paint.Style.FILL);

            // Header text paint
            headerPaint.setColor(Color.WHITE);
            headerPaint.setTextSize(36f);
            headerPaint.setAntiAlias(true);
            headerPaint.setFakeBoldText(true);

            // Header background paint
            headerBgPaint.setColor(Color.argb(150, 0, 0, 0));
            headerBgPaint.setStyle(Paint.Style.FILL);
        }

        void setOverlayText(String text) {
            overlayText.set(text);
        }

        void updateDetections(List<AprilTagDetection> detections) {
            if (detections != null) {
                currentDetections.set(new ArrayList<>(detections));
            } else {
                currentDetections.set(new ArrayList<>());
            }
        }

        @Override
        public void init(int width, int height, @Nullable CameraCalibration calibration) {
            frameWidth = width;
            frameHeight = height;
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap current = lastFrame.get();
            if (current == null || current.getWidth() != frame.width() ||
                    current.getHeight() != frame.height()) {
                if (current != null) current.recycle();
                Bitmap resized = Bitmap.createBitmap(frame.width(), frame.height(),
                        Bitmap.Config.RGB_565);
                lastFrame.set(resized);
                current = resized;
            }
            frameWidth = frame.width();
            frameHeight = frame.height();
            Utils.matToBitmap(frame, current);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {

            List<AprilTagDetection> detections = currentDetections.get();

            // Draw header with detection count at top of screen
            drawHeader(canvas, detections.size(), onscreenWidth);

            // Draw each AprilTag detection
            for (AprilTagDetection detection : detections) {
                drawAprilTagBox(canvas, detection, scaleBmpPxToCanvasPx);
            }

            // Draw custom overlay text if set
            String text = overlayText.get();
            if (text != null && !text.isEmpty()) {
                drawOverlayText(canvas, "works?", onscreenHeight);
            }
        }

        private void drawHeader(Canvas canvas, int tagCount, int canvasWidth) {
            String headerText;
            if (tagCount == 0) {
                headerText = "No AprilTags Detected";
                headerBgPaint.setColor(Color.argb(150, 100, 0, 0)); // Red-ish background
            } else if (tagCount == 1) {
                headerText = "1 AprilTag Detected";
                headerBgPaint.setColor(Color.argb(150, 0, 100, 0)); // Green-ish background
            } else {
                headerText = tagCount + " AprilTags Detected";
                headerBgPaint.setColor(Color.argb(150, 0, 100, 0)); // Green-ish background
            }

            float padding = 12f;
            float textWidth = headerPaint.measureText(headerText);
            float textHeight = headerPaint.descent() - headerPaint.ascent();

            // Center the header
            float left = (canvasWidth - textWidth) / 2f - padding;
            float top = padding;
            float right = left + textWidth + (padding * 2);
            float bottom = top + textHeight + (padding * 2);

            // Draw rounded background
            canvas.drawRoundRect(left, top, right, bottom, 10f, 10f, headerBgPaint);

            // Draw text
            canvas.drawText(headerText, left + padding,
                    top + padding - headerPaint.ascent(), headerPaint);
        }

        private void drawAprilTagBox(Canvas canvas, AprilTagDetection detection,
                                     float scaleBmpPxToCanvasPx) {

            if (detection.corners == null || detection.corners.length < 4) {
                return;
            }

            // Scale corners to canvas coordinates
            float[] scaledCornersX = new float[4];
            float[] scaledCornersY = new float[4];

            for (int i = 0; i < 4; i++) {
                scaledCornersX[i] = (float) detection.corners[i].x * scaleBmpPxToCanvasPx;
                scaledCornersY[i] = (float) detection.corners[i].y * scaleBmpPxToCanvasPx;
            }

            // Draw the bounding box as a polygon connecting corners
            Path boxPath = new Path();
            boxPath.moveTo(scaledCornersX[0], scaledCornersY[0]);
            for (int i = 1; i < 4; i++) {
                boxPath.lineTo(scaledCornersX[i], scaledCornersY[i]);
            }
            boxPath.close();
            canvas.drawPath(boxPath, boxPaint);

            // Draw corner dots
            float cornerRadius = 8f;
            for (int i = 0; i < 4; i++) {
                canvas.drawCircle(scaledCornersX[i], scaledCornersY[i], cornerRadius, cornerPaint);
            }

            // Calculate center of tag for label positioning
            float centerX = (float) detection.center.x * scaleBmpPxToCanvasPx;
            float centerY = (float) detection.center.y * scaleBmpPxToCanvasPx;

            // Find top of the tag for placing info above it
            float minY = scaledCornersY[0];
            for (int i = 1; i < 4; i++) {
                if (scaledCornersY[i] < minY) {
                    minY = scaledCornersY[i];
                }
            }

            // Build info text
            String tagIdText = "ID: " + detection.id;
            String distanceText = "";
            String positionText = "";

            if (detection.ftcPose != null) {
                // Distance in inches, convert to more readable format
                double distanceInches = detection.ftcPose.range;
                if (distanceInches < 12) {
                    distanceText = String.format("Dist: %.1f in", distanceInches);
                } else {
                    double distanceFeet = distanceInches / 12.0;
                    distanceText = String.format("Dist: %.1f ft (%.0f in)",
                            distanceFeet, distanceInches);
                }

                // X, Y, Z position
                positionText = String.format("X:%.1f Y:%.1f Z:%.1f",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z);
            }

            // Draw info box above the tag
            drawTagInfo(canvas, tagIdText, distanceText, positionText, centerX, minY);

            // Draw center crosshair
            float crossSize = 10f;
            Paint crossPaint = new Paint();
            crossPaint.setColor(Color.YELLOW);
            crossPaint.setStrokeWidth(2f);
            canvas.drawLine(centerX - crossSize, centerY,
                    centerX + crossSize, centerY, crossPaint);
            canvas.drawLine(centerX, centerY - crossSize,
                    centerX, centerY + crossSize, crossPaint);
        }

        private void drawTagInfo(Canvas canvas, String idText, String distText,
                                 String posText, float centerX, float aboveY) {

            float padding = 8f;
            float lineSpacing = 4f;
            float textHeight = textPaint.descent() - textPaint.ascent();

            // Calculate how many lines we have
            int lineCount = 1; // ID is always shown
            if (!distText.isEmpty()) lineCount++;
            if (!posText.isEmpty()) lineCount++;

            // Calculate box dimensions
            float maxWidth = textPaint.measureText(idText);
            if (!distText.isEmpty()) {
                maxWidth = Math.max(maxWidth, textPaint.measureText(distText));
            }
            if (!posText.isEmpty()) {
                maxWidth = Math.max(maxWidth, textPaint.measureText(posText));
            }

            float boxWidth = maxWidth + (padding * 2);
            float boxHeight = (textHeight * lineCount) + (lineSpacing * (lineCount - 1)) +
                    (padding * 2);

            // Position the box above the tag, centered horizontally
            float left = centerX - (boxWidth / 2);
            float bottom = aboveY - 10f; // 10px gap above tag
            float top = bottom - boxHeight;
            float right = left + boxWidth;

            // Draw background
            canvas.drawRoundRect(left, top, right, bottom, 8f, 8f, textBgPaint);

            // Draw text lines
            float textX = left + padding;
            float textY = top + padding - textPaint.ascent();

            // ID line (with color coding)
            Paint idPaint = new Paint(textPaint);
            idPaint.setColor(Color.GREEN);
            canvas.drawText(idText, textX, textY, idPaint);
            textY += textHeight + lineSpacing;

            // Distance line
            if (!distText.isEmpty()) {
                Paint distPaint = new Paint(textPaint);
                distPaint.setColor(Color.CYAN);
                canvas.drawText(distText, textX, textY, distPaint);
                textY += textHeight + lineSpacing;
            }

            // Position line
            if (!posText.isEmpty()) {
                Paint posPaint = new Paint(textPaint);
                posPaint.setColor(Color.YELLOW);
                posPaint.setTextSize(24f); // Smaller text for position
                canvas.drawText(posText, textX, textY, posPaint);
            }
        }

        private void drawOverlayText(Canvas canvas, String text, int canvasHeight) {
            float padding = 8f;
            float textHeight = headerPaint.descent() - headerPaint.ascent();
            float textWidth = headerPaint.measureText(text);

            float left = padding;
            float bottom = canvasHeight - padding;
            float top = bottom - textHeight - (padding * 2);

            canvas.drawRoundRect(left, top, left + textWidth + (padding * 2),
                    bottom, 8f, 8f, textBgPaint);
            canvas.drawText(text, left + padding,
                    bottom - padding - headerPaint.descent(), headerPaint);
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> {
                Bitmap frame = lastFrame.get();
                if (frame == null) {
                    frame = Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565);
                }
                bitmapConsumer.accept(frame);
            });
        }
    }
}