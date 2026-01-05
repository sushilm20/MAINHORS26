package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

@Configurable
public final class AutoVarsRed {

    private AutoVarsRed() {}

    /**
     * Single shared set of red-side tunables used by all red autos.
     * Update values here once to change every red OpMode.
     */
    public static final class Shared {
        public static double INTAKE_RUN_SECONDS = 2.15;
        public static double TIMED_INTAKE_SECONDS = 0.93;
        public static long CLAW_CLOSE_MS = 250L;
        public static double PRE_ACTION_WAIT_SECONDS = 0.25;
        public static double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;
        public static long SHOOTER_WAIT_TIMEOUT_MS = 3000L;
        public static double AUTO_SHOOTER_RPM = FlywheelController.TARGET_RPM_CLOSE;
        public static double INTAKE_ON_POWER = 1.0;
        public static double SHOOT_POSE_INTAKE_POWER = 0.4;
        public static double CLOSED_INTAKE_POWER = 0.35;
        public static double CLOSED_INTAKE_TOLERANCE_IN = 12.0;
        public static double LEFT_COMPRESSION_OFF = 0.5;
        public static double RIGHT_COMPRESSION_OFF = 0.5;
        public static double SHOOT_POSE_X = 96.0;
        public static double SHOOT_POSE_Y = 96.0;
        public static double START_POSE_TOLERANCE_IN = 6.0;
        public static double GATE_OPEN = 0.67;
        public static double GATE_CLOSED = 0.5;
        public static double GATE_OPEN_TOLERANCE_IN = START_POSE_TOLERANCE_IN + 3.0;
        public static double GATE_CLOSE_TOLERANCE_IN = GATE_OPEN_TOLERANCE_IN + 1.0;
    }
}
