package org.firstinspires.ftc.teamcode.tracking;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import androidx.annotation.Nullable;

import com.bylazar.camerastream.PanelsCameraStream;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

/** Shared stream/overlay helper for Panels streaming. */
public class CameraStreamManager {
    private final StreamProcessor streamProcessor = new StreamProcessor();

    public VisionProcessor getVisionProcessor() { return streamProcessor; }
    public CameraStreamSource getCameraStreamSource() { return streamProcessor; }

    public void startPanelsStream(int fps) {
        PanelsCameraStream.INSTANCE.startStream(streamProcessor, fps);
    }

    public void stopPanelsStream() {
        PanelsCameraStream.INSTANCE.stopStream();
    }

    public void setOverlayText(String text) {
        streamProcessor.setOverlayText(text);
    }

    private static class StreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(null);
        private final AtomicReference<String> overlayText = new AtomicReference<>("");
        private final Paint textPaint = new Paint();
        private final Paint bgPaint = new Paint();

        StreamProcessor() {
            textPaint.setColor(Color.WHITE);
            textPaint.setTextSize(36f);
            textPaint.setAntiAlias(true);
            bgPaint.setColor(Color.argb(120, 0, 0, 0)); // semi-transparent
        }

        void setOverlayText(String text) {
            overlayText.set(text);
        }

        @Override
        public void init(int width, int height, @Nullable CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap current = lastFrame.get();
            if (current == null || current.getWidth() != frame.width() || current.getHeight() != frame.height()) {
                if (current != null) current.recycle();
                Bitmap resized = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
                lastFrame.set(resized);
                current = resized;
            }
            Utils.matToBitmap(frame, current);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            String text = overlayText.get();
            if (text != null && !text.isEmpty()) {
                float padding = 8f;
                float textHeight = textPaint.descent() - textPaint.ascent();
                float textWidth = textPaint.measureText(text);
                float left = padding;
                float top = padding - textPaint.ascent();
                canvas.drawRect(left - padding, padding,
                        left + textWidth + padding, padding + textHeight + padding, bgPaint);
                canvas.drawText(text, left, top, textPaint);
            }
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> {
                Bitmap frame = lastFrame.get();
                if (frame == null) frame = Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565);
                bitmapConsumer.accept(frame);
            });
        }
    }
}