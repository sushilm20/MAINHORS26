package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

public final class AutoVarsBlue {

    private AutoVarsBlue() {}

    @Configurable
    public static final class Main {
        public static final double INTAKE_RUN_SECONDS = 1.4;
        public static final double TIMED_INTAKE_SECONDS = 0.93;
        public static final long CLAW_CLOSE_MS = 250L;
        public static final double PRE_ACTION_WAIT_SECONDS = 0.25;
        public static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;
        public static final long SHOOTER_WAIT_TIMEOUT_MS = 3000L;
        public static final double AUTO_SHOOTER_RPM = FlywheelController.TARGET_RPM_CLOSE;
        public static final double INTAKE_ON_POWER = 1.0;
        public static final double SHOOT_POSE_INTAKE_POWER = 0.8;
        public static final double CLOSED_INTAKE_POWER = 0.35;
        public static final double CLOSED_INTAKE_TOLERANCE_IN = 12.0;
        public static final double LEFT_COMPRESSION_OFF = 0.5;
        public static final double RIGHT_COMPRESSION_OFF = 0.5;
        public static final double SHOOT_POSE_X = 48.0;
        public static final double SHOOT_POSE_Y = 96.0;
        public static final double START_POSE_TOLERANCE_IN = 6.0;
        public static final double GATE_OPEN = 0.67;
        public static final double GATE_CLOSED = 0.5;
        public static final double GATE_OPEN_TOLERANCE_IN = START_POSE_TOLERANCE_IN + 3.0;
        public static final double GATE_CLOSE_TOLERANCE_IN = GATE_OPEN_TOLERANCE_IN + 1.0;

        private Main() {}
    }

    @Configurable
    public static final class Experimental {
        public static final double INTAKE_RUN_SECONDS = 1.0;
        public static final double TIMED_INTAKE_SECONDS = 0.93;
        public static final long CLAW_CLOSE_MS = 200L;
        public static final double PRE_ACTION_WAIT_SECONDS = 0.2;
        public static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;
        public static final long SHOOTER_WAIT_TIMEOUT_MS = 3000L;
        public static final double AUTO_SHOOTER_RPM = FlywheelController.TARGET_RPM_CLOSE;
        public static final double INTAKE_ON_POWER = 1.0;
        public static final double SHOOT_POSE_INTAKE_POWER = 0.4;
        public static final double CLOSED_INTAKE_POWER = 0.35;
        public static final double CLOSED_INTAKE_TOLERANCE_IN = 12.0;
        public static final double LEFT_COMPRESSION_OFF = 0.5;
        public static final double RIGHT_COMPRESSION_OFF = 0.5;
        public static final double SHOOT_POSE_X = 48.0;
        public static final double SHOOT_POSE_Y = 96.0;
        public static final double START_POSE_TOLERANCE_IN = 6.0;
        public static final double GATE_OPEN = 0.67;
        public static final double GATE_CLOSED = 0.5;
        public static final double GATE_OPEN_TOLERANCE_IN = START_POSE_TOLERANCE_IN + 3.0;
        public static final double GATE_CLOSE_TOLERANCE_IN = GATE_OPEN_TOLERANCE_IN + 1.0;

        private Experimental() {}
    }

    @Configurable
    public static final class Config {
        @Sorter(sort = 0)
        public static double INTAKE_RUN_SECONDS = 1.5;
        @Sorter(sort = 1)
        public static double TIMED_INTAKE_SECONDS = 0.93;
        @Sorter(sort = 2)
        public static long CLAW_CLOSE_MS = 250L;
        @Sorter(sort = 3)
        public static double PRE_ACTION_WAIT_SECONDS = 0.25;
        @Sorter(sort = 4)
        public static double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;
        @Sorter(sort = 5)
        public static long SHOOTER_WAIT_TIMEOUT_MS = 3000L;
        @Sorter(sort = 6)
        public static double AUTO_SHOOTER_RPM = FlywheelController.TARGET_RPM_CLOSE;
        @Sorter(sort = 7)
        public static double INTAKE_ON_POWER = 1.0;
        @Sorter(sort = 8)
        public static double SHOOT_POSE_INTAKE_POWER = 0.4;
        @Sorter(sort = 9)
        public static double CLOSED_INTAKE_POWER = 0.35;
        @Sorter(sort = 10)
        public static double CLOSED_INTAKE_TOLERANCE_IN = 12.0;
        @Sorter(sort = 13)
        public static double SHOOT_POSE_X = 48.0;
        @Sorter(sort = 14)
        public static double SHOOT_POSE_Y = 96.0;
        @Sorter(sort = 15)
        public static double START_POSE_TOLERANCE_IN = 6.0;
        @Sorter(sort = 16)
        public static double GATE_OPEN = 0.67;
        @Sorter(sort = 17)
        public static double GATE_CLOSED = 0.5;
        @Sorter(sort = 18)
        public static double GATE_OPEN_TOLERANCE_IN = START_POSE_TOLERANCE_IN + 3.0;
        @Sorter(sort = 19)
        public static double GATE_CLOSE_TOLERANCE_IN = GATE_OPEN_TOLERANCE_IN + 1.0;

        private Config() {}
    }

    @Configurable
    public static final class Legacy {
        public static final double INTAKE_WAIT_SECONDS = 2.5;
        public static final double TIMED_INTAKE_SECONDS = 1.0;
        public static final long CLAW_CLOSE_MS = 250L;
        public static final double PRE_ACTION_WAIT_SECONDS = 0.3;
        public static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;
        public static final long SHOOTER_WAIT_TIMEOUT_MS = 4000L;
        public static final double AUTO_SHOOTER_RPM = FlywheelController.TARGET_RPM_CLOSE;
        public static final double INTAKE_ON_POWER = 1.0;
        public static final double LEFT_COMPRESSION_ON = 1.0;
        public static final double RIGHT_COMPRESSION_ON = 0.0;
        public static final double LEFT_COMPRESSION_OFF = 0.5;
        public static final double RIGHT_COMPRESSION_OFF = 0.5;
        public static final double SHOOT_POSE_X = 48.0;
        public static final double SHOOT_POSE_Y = 96.0;
        public static final double START_POSE_TOLERANCE_IN = 6.0;

        private Legacy() {}
    }
}
