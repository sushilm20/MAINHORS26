package org.firstinspires.ftc.teamcode.extras;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

/**
 * Centralised telemetry helper that can be shared across different TeleOps
 * (e.g. OfficialHORS, AdaptiveHORS, DynamicTrackingTurret).
 *
 * Instead of each TeleOp pulling data from controllers and formatting it
 * inline, this class owns the display logic so it stays consistent and
 * reusable.
 */
public class TelemetryData {

    private final Telemetry telemetry;
    private final FlywheelController flywheel;

    // Loop-time tracking (owned here so TeleOps don't duplicate it)
    private long lastLoopTimeMs;
    private double loopTimeMs = 0.0;
    private double avgLoopTimeMs = 0.0;
    private long loopCount = 0;

    // Pose snapshot set each loop by the TeleOp
    private Pose currentPose = null;

    public TelemetryData(Telemetry telemetry, FlywheelController flywheel) {
        this.telemetry = telemetry;
        this.flywheel = flywheel;
        this.lastLoopTimeMs = System.currentTimeMillis();
    }

    /** Call once at the top of each loop iteration to track loop timing. */
    public void updateLoopTime() {
        long nowMs = System.currentTimeMillis();
        loopTimeMs = nowMs - lastLoopTimeMs;
        lastLoopTimeMs = nowMs;
        loopCount++;
        if (loopCount > 1) {
            avgLoopTimeMs += (loopTimeMs - avgLoopTimeMs) / (loopCount - 1);
        }
    }

    /** Set the current robot pose (from Follower or any other source). */
    public void setPose(Pose pose) {
        this.currentPose = pose;
    }

    /**
     * Push all standard telemetry lines and call {@code telemetry.update()}.
     * TeleOps may add extra lines via {@link Telemetry#addData} <i>before</i>
     * calling this method; they will appear alongside the standard data.
     */
    public void update() {
        telemetry.addData("Flywheel", "Current: %.0f rpm | Target: %.0f rpm",
                flywheel.getCurrentRPM(), flywheel.getTargetRPM());

        telemetry.addData("Pose", currentPose != null
                ? String.format("(%.1f, %.1f, %.1f°)",
                        currentPose.getX(), currentPose.getY(),
                        Math.toDegrees(currentPose.getHeading()))
                : "N/A");

        telemetry.addData("Loop", "%.0f ms (%.0f Hz) | avg %.1f ms (%.0f Hz)",
                loopTimeMs,
                loopTimeMs > 0 ? 1000.0 / loopTimeMs : 0,
                avgLoopTimeMs,
                avgLoopTimeMs > 0 ? 1000.0 / avgLoopTimeMs : 0);

        telemetry.update();
    }

    // ── Getters (useful if a TeleOp needs the raw values) ──

    public double getLoopTimeMs()    { return loopTimeMs; }
    public double getAvgLoopTimeMs() { return avgLoopTimeMs; }
    public long   getLoopCount()     { return loopCount; }
    public Pose   getCurrentPose()   { return currentPose; }
}
