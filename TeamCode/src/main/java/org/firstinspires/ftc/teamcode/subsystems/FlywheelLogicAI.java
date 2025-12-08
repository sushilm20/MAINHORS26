package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelLogicAI {

    private final Flywheel flywheel; // RPM logic
    private final DcMotor intakeMotor;
    private final Servo leftCompressionServo;
    private final Servo rightCompressionServo;

    // Feed constants
    private static final double INTAKE_POWER = 1.0;
    private static final double SERVO_FEED_LEFT = 1.0;
    private static final double SERVO_FEED_RIGHT = 0.0;
    private static final double SERVO_IDLE = 0.5;

    // Timing
    private long stateStartMs = 0L;
    private int shotsRemaining = 0;

    private enum State { IDLE, SPINUP, FEED, RESET }
    private State state = State.IDLE;

    private static final long MAX_SPINUP_MS = 2000;
    private static final long FEED_MS = 500;
    private static final long RESET_MS = 400;

    // Constructor — FIXED
    public FlywheelLogicAI(DcMotor shooterMotor,
                           DcMotor intakeMotor,
                           Servo leftCompressionServo,
                           Servo rightCompressionServo,
                           Telemetry telemetry) {
        this.flywheel = new Flywheel(shooterMotor, telemetry);
        this.intakeMotor = intakeMotor;
        this.leftCompressionServo = leftCompressionServo;
        this.rightCompressionServo = rightCompressionServo;

        // Initial positions
        intakeMotor.setPower(0.0);
        leftCompressionServo.setPosition(SERVO_IDLE);
        rightCompressionServo.setPosition(SERVO_IDLE);

        flywheel.setTargetRPM(90.0); // default close-range RPM
    }

    /** Call every loop */
    public void update(long nowMs, boolean calibPressed) {
        flywheel.update(nowMs, calibPressed);

        switch (state) {
            case IDLE:
                if (shotsRemaining > 0) {
                    flywheel.setShooterOn(true);
                    stateStartMs = nowMs;
                    state = State.SPINUP;
                }
                break;

            case SPINUP:
                if (flywheel.isAtTarget() || (nowMs - stateStartMs > MAX_SPINUP_MS)) {
                    intakeMotor.setPower(INTAKE_POWER);
                    leftCompressionServo.setPosition(SERVO_FEED_LEFT);
                    rightCompressionServo.setPosition(SERVO_FEED_RIGHT);
                    stateStartMs = nowMs;
                    state = State.FEED;
                }
                break;

            case FEED:
                if (nowMs - stateStartMs > FEED_MS) {
                    shotsRemaining--;
                    intakeMotor.setPower(0.0);
                    leftCompressionServo.setPosition(SERVO_IDLE);
                    rightCompressionServo.setPosition(SERVO_IDLE);
                    stateStartMs = nowMs;
                    state = State.RESET;
                }
                break;

            case RESET:
                if (nowMs - stateStartMs > RESET_MS) {
                    if (shotsRemaining > 0) {
                        stateStartMs = nowMs;
                        state = State.SPINUP;
                    } else {
                        flywheel.setShooterOn(false);
                        state = State.IDLE;
                    }
                }
                break;
        }
    }

    /** Queue shots to fire */
    public void fireShots(int numShots) {
        if (state == State.IDLE) {
            shotsRemaining = numShots;
        }
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    public double getCurrentRPM() {
        return flywheel.getCurrentRPM();
    }
}
