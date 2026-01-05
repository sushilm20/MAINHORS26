package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

@Configurable
public final class AutoVarsRed {

    private AutoVarsRed() {}

    public static final double INTAKE_RUN_SECONDS = 2.15;
    public static final double TIMED_INTAKE_SECONDS = 0.93;
    public static final long CLAW_CLOSE_MS = 250L;
    public static final double PRE_ACTION_WAIT_SECONDS = 0.25;
    public static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;
    public static final long SHOOTER_WAIT_TIMEOUT_MS = 3000L;
    public static final double AUTO_SHOOTER_RPM = FlywheelController.TARGET_RPM_CLOSE;
    public static final double INTAKE_ON_POWER = 1.0;
    public static final double SHOOT_POSE_INTAKE_POWER = 0.4;
    public static final double CLOSED_INTAKE_POWER = 0.35;
    public static final double CLOSED_INTAKE_TOLERANCE_IN = 12.0;
    public static final double LEFT_COMPRESSION_OFF = 0.5;
    public static final double RIGHT_COMPRESSION_OFF = 0.5;
    public static final double SHOOT_POSE_X = 96.0;
    public static final double SHOOT_POSE_Y = 96.0;
    public static final double START_POSE_TOLERANCE_IN = 6.0;
    public static final double GATE_OPEN = 0.67;
    public static final double GATE_CLOSED = 0.5;
    public static final double GATE_OPEN_TOLERANCE_IN = START_POSE_TOLERANCE_IN + 3.0;
    public static final double GATE_CLOSE_TOLERANCE_IN = GATE_OPEN_TOLERANCE_IN + 1.0;
}
