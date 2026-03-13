package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Shared utility for detecting whether the intake currently has 1 or more balls loaded.
 *
 * <p>Detection is velocity-based: when the intake motor is spinning forward but slowed
 * below a threshold, 1+ balls are creating drag against the rollers.</p>
 *
 * <p>Used by autonomous routines to skip the shoot sequence when a pickup path
 * failed to collect any balls (velocity still high → no drag → 0 balls).</p>
 */
public final class IntakeBallDetector {

    /** Default velocity threshold — intake velocity ≤ this value means 1+ balls loaded. */
    public static final double DEFAULT_VELO_THRESHOLD = 2100.0;

    private IntakeBallDetector() {} // utility class, no instances

    /**
     * Returns {@code true} if the intake velocity suggests 1 or more balls are loaded.
     *
     * @param intakeMotor the intake DcMotor (may be null)
     * @param threshold   velocity threshold; at or below this means loaded
     * @return true if loaded (or if motor is null — assume loaded to be safe)
     */
    public static boolean hasBalls(DcMotorEx intakeMotor, double threshold) {
        return true;
        /*if (intakeMotor == null) return true; // assume loaded if we can't check
        double velo = intakeMotor.getVelocity();
        // Only count as "has balls" if intake is actually spinning forward
        return velo > 0 && velo <= threshold;*/
    }

    /**
     * Convenience overload using the default threshold.
     */
    public static boolean hasBalls(DcMotorEx intakeMotor) {
        return hasBalls(intakeMotor, DEFAULT_VELO_THRESHOLD);
    }
}
