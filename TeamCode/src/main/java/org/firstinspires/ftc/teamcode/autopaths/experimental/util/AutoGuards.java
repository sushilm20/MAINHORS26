package org.firstinspires.ftc.teamcode.autopaths.experimental.util;

import com.pedropathing.geometry.Pose;

public final class AutoGuards {
    private AutoGuards() {}

    public static boolean hasTimedOut(long startMs, long timeoutMs, long nowMs) {
        if (startMs < 0) return false;
        return nowMs - startMs >= timeoutMs;
    }

    public static boolean shouldForcePark(long autoStartMs, long nowMs, long cutoffMs) {
        if (autoStartMs < 0) return false;
        return nowMs - autoStartMs >= cutoffMs;
    }

    public static double distance(Pose a, Pose b) {
        if (a == null || b == null) return Double.POSITIVE_INFINITY;
        double dx = a.getX() - b.getX();
        double dy = a.getY() - b.getY();
        return Math.hypot(dx, dy);
    }

    public static boolean isMotionStable(double translationalSpeedInPerS,
                                         double angularSpeedDegPerS,
                                         double maxTransSpeed,
                                         double maxAngSpeed) {
        return translationalSpeedInPerS <= maxTransSpeed &&
                angularSpeedDegPerS <= maxAngSpeed;
    }

    public static final class ShooterGate {
        private final double poseTolIn;
        private final double rpmTol;
        private final double maxTransSpeed;
        private final double maxAngSpeed;
        private final long stableHoldMs;

        private long stableStartMs = -1L;

        public ShooterGate(double poseTolIn,
                           double rpmTol,
                           double maxTransSpeed,
                           double maxAngSpeed,
                           long stableHoldMs) {
            this.poseTolIn = poseTolIn;
            this.rpmTol = rpmTol;
            this.maxTransSpeed = maxTransSpeed;
            this.maxAngSpeed = maxAngSpeed;
            this.stableHoldMs = stableHoldMs;
        }

        public void reset() {
            stableStartMs = -1L;
        }

        public boolean isReady(long nowMs,
                               Pose currentPose,
                               Pose shootPose,
                               double currentRpm,
                               double targetRpm,
                               double translationalSpeedInPerS,
                               double angularSpeedDegPerS) {

            boolean poseOk = distance(currentPose, shootPose) <= poseTolIn;
            boolean rpmOk = Math.abs(targetRpm - currentRpm) <= rpmTol;
            boolean motionOk = isMotionStable(
                    translationalSpeedInPerS, angularSpeedDegPerS,
                    maxTransSpeed, maxAngSpeed
            );

            boolean allOk = poseOk && rpmOk && motionOk;

            if (!allOk) {
                stableStartMs = -1L;
                return false;
            }

            if (stableStartMs < 0) stableStartMs = nowMs;
            return nowMs - stableStartMs >= stableHoldMs;
        }
    }

    public static final class JamDetector {
        private final double jamVelocityThreshold;
        private final long jamDetectMs;
        private final long clearReverseMs;
        private final double clearReversePower;

        private long lowVelocityStartMs = -1L;
        private long clearStartMs = -1L;
        private boolean clearing = false;

        public JamDetector(double jamVelocityThreshold,
                           long jamDetectMs,
                           long clearReverseMs,
                           double clearReversePower) {
            this.jamVelocityThreshold = jamVelocityThreshold;
            this.jamDetectMs = jamDetectMs;
            this.clearReverseMs = clearReverseMs;
            this.clearReversePower = clearReversePower;
        }

        public void reset() {
            lowVelocityStartMs = -1L;
            clearStartMs = -1L;
            clearing = false;
        }

        public double update(long nowMs, double desiredIntakePower, double measuredVelocity) {
            if (clearing) {
                if (nowMs - clearStartMs >= clearReverseMs) {
                    clearing = false;
                    lowVelocityStartMs = -1L;
                } else {
                    return clearReversePower;
                }
            }

            boolean intakeForwardCommanded = desiredIntakePower < -0.05;

            if (!intakeForwardCommanded) {
                lowVelocityStartMs = -1L;
                return desiredIntakePower;
            }

            boolean lowVel = Math.abs(measuredVelocity) <= jamVelocityThreshold;

            if (!lowVel) {
                lowVelocityStartMs = -1L;
                return desiredIntakePower;
            }

            if (lowVelocityStartMs < 0) {
                lowVelocityStartMs = nowMs;
                return desiredIntakePower;
            }

            if (nowMs - lowVelocityStartMs >= jamDetectMs) {
                clearing = true;
                clearStartMs = nowMs;
                return clearReversePower;
            }

            return desiredIntakePower;
        }

        public boolean isClearing() {
            return clearing;
        }
    }
}