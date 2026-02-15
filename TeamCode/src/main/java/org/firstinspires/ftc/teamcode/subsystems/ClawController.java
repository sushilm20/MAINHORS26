package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawController {
    public static final double CLAW_OPEN = 0.6;
    public static final double CLAW_CLOSED = 0.3;
    public static final long CLAW_CLOSE_MS = 500L;

    private final Servo claw;
    private final double openPos;
    private final double closedPos;
    private final long closeMs;

    private boolean active = false;
    private long startMs = 0L;

    public ClawController(Servo claw, double openPos, double closedPos, long closeMs) {
        this.claw = claw;
        this.openPos = openPos;
        this.closedPos = closedPos;
        this.closeMs = closeMs;
        this.claw.setPosition(openPos);
    }

    public void trigger(long nowMs) {
        claw.setPosition(closedPos);
        active = true;
        startMs = nowMs;
    }

    public void update(long nowMs) {
        if (active && nowMs >= startMs + closeMs) {
            claw.setPosition(openPos);
            active = false;
        }
    }
}