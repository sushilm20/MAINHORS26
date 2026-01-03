package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

public class GateController {
    public enum GateCycleState { IDLE, OPEN_INTAKE }

    private final Servo gateServo;
    private final DcMotor intakeMotor;
    private final LED led1Red, led1Green, led2Red, led2Green;
    private final double gateOpenPos, gateClosedPos;
    private final long intakeDurationMs;
    private final long clawTriggerBeforeEndMs;
    private final double intakeSequencePower;

    private boolean gateClosed = false;
    private GateCycleState state = GateCycleState.IDLE;
    private long startMs = 0L;
    private boolean clawTriggerSent = false;

    public GateController(Servo gateServo,
                          DcMotor intakeMotor,
                          LED led1Red, LED led1Green,
                          LED led2Red, LED led2Green,
                          double gateOpenPos, double gateClosedPos,
                          long intakeDurationMs, long clawTriggerBeforeEndMs,
                          double intakeSequencePower) {
        this.gateServo = gateServo;
        this.intakeMotor = intakeMotor;
        this.led1Red = led1Red;
        this.led1Green = led1Green;
        this.led2Red = led2Red;
        this.led2Green = led2Green;
        this.gateOpenPos = gateOpenPos;
        this.gateClosedPos = gateClosedPos;
        this.intakeDurationMs = intakeDurationMs;
        this.clawTriggerBeforeEndMs = clawTriggerBeforeEndMs;
        this.intakeSequencePower = intakeSequencePower;
    }

    public void setGateClosed(boolean closed) {
        gateClosed = closed;
        gateServo.setPosition(gateClosed ? gateClosedPos : gateOpenPos);
        updateLeds();
    }

    public void toggleGate() {
        if (state == GateCycleState.IDLE) {
            setGateClosed(!gateClosed);
        }
    }

    /** Returns true once when it’s time to trigger the claw during the auto sequence. */
    public boolean update(long nowMs) {
        boolean shouldTriggerClaw = false;
        if (state == GateCycleState.OPEN_INTAKE) {
            long elapsed = nowMs - startMs;
            long remaining = intakeDurationMs - elapsed;

            if (remaining <= clawTriggerBeforeEndMs && !clawTriggerSent) {
                clawTriggerSent = true;
                shouldTriggerClaw = true;
            }
            if (elapsed >= intakeDurationMs) {
                intakeMotor.setPower(0.0);
                setGateClosed(true);
                state = GateCycleState.IDLE;
            }
        }
        return shouldTriggerClaw;
    }

    public void startIntakeSequence(long nowMs) {
        if (state != GateCycleState.IDLE) return;
        setGateClosed(false);
        if (intakeMotor != null) {
            intakeMotor.setPower(intakeSequencePower);
        }
        startMs = nowMs;
        state = GateCycleState.OPEN_INTAKE;
        clawTriggerSent = false;
    }

    public boolean isBusy() {
        return state != GateCycleState.IDLE;
    }

    public boolean isGateClosed() {
        return gateClosed;
    }

    public GateCycleState getState() {
        return state;
    }

    private void updateLeds() {
        boolean gateOpen = !gateClosed;
        setLed(led1Red, gateOpen);
        setLed(led2Red, gateOpen);
        setLed(led1Green, !gateOpen);
        setLed(led2Green, !gateOpen);
    }

    private void setLed(LED led, boolean on) {
        if (led == null) return;
        if (on) led.on(); else led.off();
    }
}