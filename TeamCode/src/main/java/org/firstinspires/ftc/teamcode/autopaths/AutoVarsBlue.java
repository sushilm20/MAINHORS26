package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

public final class AutoVarsBlue {

    private AutoVarsBlue() {}

    /**
     * Single shared set of blue-side tunables used by all blue autos.
     * Update values here once to change every blue OpMode.
     */
    @Configurable
    public static final class Shared {
        public static double INTAKE_RUN_SECONDS = 1.5;
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
        public static double LEFT_COMPRESSION_ON = 1.0;
        public static double RIGHT_COMPRESSION_ON = 0.0;
        public static double LEFT_COMPRESSION_OFF = 0.5;
        public static double RIGHT_COMPRESSION_OFF = 0.5;
        public static double SHOOT_POSE_X = 48.0;
        public static double SHOOT_POSE_Y = 96.0;
        public static double START_POSE_TOLERANCE_IN = 6.0;
        public static double GATE_OPEN = 0.67;
        public static double GATE_CLOSED = 0.5;
        public static double GATE_OPEN_TOLERANCE_IN = START_POSE_TOLERANCE_IN + 3.0;
        public static double GATE_CLOSE_TOLERANCE_IN = GATE_OPEN_TOLERANCE_IN + 1.0;
        public static double INTAKE_WAIT_SECONDS = 2.5;
    }

    // Legacy nested classes retained for reference but no longer used.
    @Configurable
    public static final class Main {
        public static final double INTAKE_RUN_SECONDS = Shared.INTAKE_RUN_SECONDS;
        public static final double TIMED_INTAKE_SECONDS = Shared.TIMED_INTAKE_SECONDS;
        public static final long CLAW_CLOSE_MS = Shared.CLAW_CLOSE_MS;
        public static final double PRE_ACTION_WAIT_SECONDS = Shared.PRE_ACTION_WAIT_SECONDS;
        public static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = Shared.PRE_ACTION_MAX_POSE_WAIT_SECONDS;
        public static final long SHOOTER_WAIT_TIMEOUT_MS = Shared.SHOOTER_WAIT_TIMEOUT_MS;
        public static final double AUTO_SHOOTER_RPM = Shared.AUTO_SHOOTER_RPM;
        public static final double INTAKE_ON_POWER = Shared.INTAKE_ON_POWER;
        public static final double SHOOT_POSE_INTAKE_POWER = Shared.SHOOT_POSE_INTAKE_POWER;
        public static final double CLOSED_INTAKE_POWER = Shared.CLOSED_INTAKE_POWER;
        public static final double CLOSED_INTAKE_TOLERANCE_IN = Shared.CLOSED_INTAKE_TOLERANCE_IN;
        public static final double LEFT_COMPRESSION_OFF = Shared.LEFT_COMPRESSION_OFF;
        public static final double RIGHT_COMPRESSION_OFF = Shared.RIGHT_COMPRESSION_OFF;
        public static final double SHOOT_POSE_X = Shared.SHOOT_POSE_X;
        public static final double SHOOT_POSE_Y = Shared.SHOOT_POSE_Y;
        public static final double START_POSE_TOLERANCE_IN = Shared.START_POSE_TOLERANCE_IN;
        public static final double GATE_OPEN = Shared.GATE_OPEN;
        public static final double GATE_CLOSED = Shared.GATE_CLOSED;
        public static final double GATE_OPEN_TOLERANCE_IN = Shared.GATE_OPEN_TOLERANCE_IN;
        public static final double GATE_CLOSE_TOLERANCE_IN = Shared.GATE_CLOSE_TOLERANCE_IN;
        private Main() {}
    }

    @Configurable
    public static final class Experimental {
        public static final double INTAKE_RUN_SECONDS = Shared.INTAKE_RUN_SECONDS;
        public static final double TIMED_INTAKE_SECONDS = Shared.TIMED_INTAKE_SECONDS;
        public static final long CLAW_CLOSE_MS = Shared.CLAW_CLOSE_MS;
        public static final double PRE_ACTION_WAIT_SECONDS = Shared.PRE_ACTION_WAIT_SECONDS;
        public static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = Shared.PRE_ACTION_MAX_POSE_WAIT_SECONDS;
        public static final long SHOOTER_WAIT_TIMEOUT_MS = Shared.SHOOTER_WAIT_TIMEOUT_MS;
        public static final double AUTO_SHOOTER_RPM = Shared.AUTO_SHOOTER_RPM;
        public static final double INTAKE_ON_POWER = Shared.INTAKE_ON_POWER;
        public static final double SHOOT_POSE_INTAKE_POWER = Shared.SHOOT_POSE_INTAKE_POWER;
        public static final double CLOSED_INTAKE_POWER = Shared.CLOSED_INTAKE_POWER;
        public static final double CLOSED_INTAKE_TOLERANCE_IN = Shared.CLOSED_INTAKE_TOLERANCE_IN;
        public static final double LEFT_COMPRESSION_OFF = Shared.LEFT_COMPRESSION_OFF;
        public static final double RIGHT_COMPRESSION_OFF = Shared.RIGHT_COMPRESSION_OFF;
        public static final double SHOOT_POSE_X = Shared.SHOOT_POSE_X;
        public static final double SHOOT_POSE_Y = Shared.SHOOT_POSE_Y;
        public static final double START_POSE_TOLERANCE_IN = Shared.START_POSE_TOLERANCE_IN;
        public static final double GATE_OPEN = Shared.GATE_OPEN;
        public static final double GATE_CLOSED = Shared.GATE_CLOSED;
        public static final double GATE_OPEN_TOLERANCE_IN = Shared.GATE_OPEN_TOLERANCE_IN;
        public static final double GATE_CLOSE_TOLERANCE_IN = Shared.GATE_CLOSE_TOLERANCE_IN;
        private Experimental() {}
    }

    @Configurable
    public static final class Config {
        @Sorter(sort = 0)
        public static double INTAKE_RUN_SECONDS = Shared.INTAKE_RUN_SECONDS;
        @Sorter(sort = 1)
        public static double TIMED_INTAKE_SECONDS = Shared.TIMED_INTAKE_SECONDS;
        @Sorter(sort = 2)
        public static long CLAW_CLOSE_MS = Shared.CLAW_CLOSE_MS;
        @Sorter(sort = 3)
        public static double PRE_ACTION_WAIT_SECONDS = Shared.PRE_ACTION_WAIT_SECONDS;
        @Sorter(sort = 4)
        public static double PRE_ACTION_MAX_POSE_WAIT_SECONDS = Shared.PRE_ACTION_MAX_POSE_WAIT_SECONDS;
        @Sorter(sort = 5)
        public static long SHOOTER_WAIT_TIMEOUT_MS = Shared.SHOOTER_WAIT_TIMEOUT_MS;
        @Sorter(sort = 6)
        public static double AUTO_SHOOTER_RPM = Shared.AUTO_SHOOTER_RPM;
        @Sorter(sort = 7)
        public static double INTAKE_ON_POWER = Shared.INTAKE_ON_POWER;
        @Sorter(sort = 8)
        public static double SHOOT_POSE_INTAKE_POWER = Shared.SHOOT_POSE_INTAKE_POWER;
        @Sorter(sort = 9)
        public static double CLOSED_INTAKE_POWER = Shared.CLOSED_INTAKE_POWER;
        @Sorter(sort = 10)
        public static double CLOSED_INTAKE_TOLERANCE_IN = Shared.CLOSED_INTAKE_TOLERANCE_IN;
        @Sorter(sort = 13)
        public static double SHOOT_POSE_X = Shared.SHOOT_POSE_X;
        @Sorter(sort = 14)
        public static double SHOOT_POSE_Y = Shared.SHOOT_POSE_Y;
        @Sorter(sort = 15)
        public static double START_POSE_TOLERANCE_IN = Shared.START_POSE_TOLERANCE_IN;
        @Sorter(sort = 16)
        public static double GATE_OPEN = Shared.GATE_OPEN;
        @Sorter(sort = 17)
        public static double GATE_CLOSED = Shared.GATE_CLOSED;
        @Sorter(sort = 18)
        public static double GATE_OPEN_TOLERANCE_IN = Shared.GATE_OPEN_TOLERANCE_IN;
        @Sorter(sort = 19)
        public static double GATE_CLOSE_TOLERANCE_IN = Shared.GATE_CLOSE_TOLERANCE_IN;
        private Config() {}
    }

    @Configurable
    public static final class Legacy {
        public static final double INTAKE_WAIT_SECONDS = Shared.INTAKE_WAIT_SECONDS;
        public static final double TIMED_INTAKE_SECONDS = Shared.TIMED_INTAKE_SECONDS;
        public static final long CLAW_CLOSE_MS = Shared.CLAW_CLOSE_MS;
        public static final double PRE_ACTION_WAIT_SECONDS = Shared.PRE_ACTION_WAIT_SECONDS;
        public static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = Shared.PRE_ACTION_MAX_POSE_WAIT_SECONDS;
        public static final long SHOOTER_WAIT_TIMEOUT_MS = Shared.SHOOTER_WAIT_TIMEOUT_MS;
        public static final double AUTO_SHOOTER_RPM = Shared.AUTO_SHOOTER_RPM;
        public static final double INTAKE_ON_POWER = Shared.INTAKE_ON_POWER;
        public static final double LEFT_COMPRESSION_ON = Shared.LEFT_COMPRESSION_ON;
        public static final double RIGHT_COMPRESSION_ON = Shared.RIGHT_COMPRESSION_ON;
        public static final double LEFT_COMPRESSION_OFF = Shared.LEFT_COMPRESSION_OFF;
        public static final double RIGHT_COMPRESSION_OFF = Shared.RIGHT_COMPRESSION_OFF;
        public static final double SHOOT_POSE_X = Shared.SHOOT_POSE_X;
        public static final double SHOOT_POSE_Y = Shared.SHOOT_POSE_Y;
        public static final double START_POSE_TOLERANCE_IN = Shared.START_POSE_TOLERANCE_IN;
        private Legacy() {}
    }
}
