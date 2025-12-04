package org.firstinspires.ftc.teamcode.autopaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="HORS Far auto", group="Linear OpMode")
public class HORSfarauto extends LinearOpMode {

    // Drive motors (same names as teleop)
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    // Mechanism motors/servos (same as teleop)
    private DcMotor shooter, turret, intakeMotor;
    private Servo clawServo, leftCompressionServo, rightCompressionServo;
    private Servo leftHoodServo, rightHoodServo;

    private ElapsedTime runtime = new ElapsedTime();

    // Shooter control state & tuning (copied from your teleop)
    private boolean shooterOn = false; // start OFF until after initial short drive
    private static final double MAX_RPM = 200.0; // keep consistent with your teleop
    private static final double TICKS_PER_REV = 537.6;
    private double currentRPM = 0.0;
    private int lastShooterPosition = 0;
    private long lastShooterTime = 0L;

    // Autonomous target for this routine
    private double targetRPM = 180.0; // requested 180 RPM
    private double kP = 0.0003;
    private double emaAlpha = 0.15;

    // Scaling and calibration (kept from your teleop)
    private double rpmScale = 0.78;
    private double ffGain = 0.8;
    private static final double MAX_SHOOTER_POWER = 0.9;

    // Hood/claw and turret settings
    private double leftHoodPosition = 0.12;
    private double rightHoodPosition = 0.12;
    private static final long CLAW_CLOSE_MS = 500L;
    private static final int TURRET_TARGET_TOLERANCE = 5;
    private static final double TARGET_TOLERANCE_RPM = 5.0; // tolerance used to detect "at target" condition

    @Override
    public void runOpMode() {
        // Hardware map (names must match your robot configuration)
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        turret = hardwareMap.get(DcMotor.class, "turret");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        leftCompressionServo = hardwareMap.get(Servo.class, "leftCompressionServo");
        rightCompressionServo = hardwareMap.get(Servo.class, "rightCompressionServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");

        // Directions & modes matching teleop setup
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initial servo positions
        clawServo.setPosition(0.63);
        leftCompressionServo.setPosition(0.5);
        rightCompressionServo.setPosition(0.5);
        leftHoodServo.setPosition(leftHoodPosition);
        rightHoodServo.setPosition(rightHoodPosition);

        telemetry.addData("Status", "Initialized - waiting for start");
        telemetry.update();

        // initialize shooter encoder/time tracking
        lastShooterPosition = shooter.getCurrentPosition();
        lastShooterTime = System.currentTimeMillis();

        // Wait for start
        waitForStart();

        // Sequence:
        // 1) Drive forward for 0.3s
        setDrivePower(0.5, 0.5);
        holdForSeconds(0.3);

        // short stop
        setDrivePower(0.0, 0.0);
        holdForSeconds(0.05);

        // 2) Turn shooter ON at 180 RPM and start regulating
        targetRPM = 180.0;
        shooterOn = true;
        // allow shooter to begin to spin while turret moves
        holdForSeconds(0.2);

        // 3) Move turret to encoder position -200 (blocking until reached or timeout)
        moveTurretToPosition(-200, 3.0); // 3s timeout

        // 4) Adjust right hood to target angle
        rightHoodPosition = 0.24;
        rightHoodServo.setPosition(rightHoodPosition);
        holdForSeconds(0.15);

        // 5) Intake automatic sequence while shooter stays ON:
        // Sequence: spin 1s, then PAUSE up to 1s OR until shooter reaches RPM (whichever happens first),
        // then spin 1s, then spin 1s, then claw action

        // Move #1: intake ON 1s
        leftCompressionServo.setPosition(1.0);
        rightCompressionServo.setPosition(0.0);
        intakeMotor.setPower(1.0);
        holdForSeconds(1.0);

        // Pause: stop intake and wait up to 1s OR until shooter within tolerance
        intakeMotor.setPower(0.0);
        leftCompressionServo.setPosition(0.5);
        rightCompressionServo.setPosition(0.5);
        // wait up to 1.0s but proceed early if shooter reaches targetRPM ± tolerance
        waitUntilAtTargetOrTimeout(1.0);

        // Move #2: intake ON 1s
        leftCompressionServo.setPosition(1.0);
        rightCompressionServo.setPosition(0.0);
        intakeMotor.setPower(1.0);
        holdForSeconds(1.0);

        // Move #3: intake ON 1s (immediately after #2)
        leftCompressionServo.setPosition(1.0);
        rightCompressionServo.setPosition(0.0);
        intakeMotor.setPower(1.0);
        holdForSeconds(1.0);

        // Stop intake and reset compression servos
        intakeMotor.setPower(0.0);
        leftCompressionServo.setPosition(0.5);
        rightCompressionServo.setPosition(0.5);
        holdForSeconds(0.05);

        // Claw action: close then reopen (simulate X button)
        clawServo.setPosition(0.2); // close
        holdForSeconds(CLAW_CLOSE_MS / 1000.0);
        clawServo.setPosition(0.63); // reopen

        // 6) After those actions, move forward briefly (duration chosen 0.5s)
        setDrivePower(0.5, 0.5);
        holdForSeconds(0.5);

        // Stop drive
        setDrivePower(0.0, 0.0);
        holdForSeconds(0.05);

        // 7) Turn off shooter and ensure all mechanisms stopped
        shooterOn = false;
        shooter.setPower(0.0);
        intakeMotor.setPower(0.0);
        turret.setPower(0.0);
        setDrivePower(0.0, 0.0);

        telemetry.addData("Auto", "FarBall sequence complete");
        telemetry.update();

        // final telemetry hold
        holdForSeconds(0.5);
    }

    // Helper: update shooter regulation using same logic as teleop
    private void updateShooter(long nowMs) {
        int currentPosition = shooter.getCurrentPosition();
        int deltaTicks = currentPosition - lastShooterPosition;
        long deltaTimeMs = nowMs - lastShooterTime;
        if (deltaTimeMs <= 0) deltaTimeMs = 1;
        double ticksPerSec = (deltaTicks * 1000.0) / deltaTimeMs;
        double measuredRPMRaw = (ticksPerSec / TICKS_PER_REV) * 60.0;
        double measuredRPMScaled = measuredRPMRaw * rpmScale;

        // EMA smoothing
        currentRPM = (1.0 - emaAlpha) * currentRPM + emaAlpha * measuredRPMScaled;
        if (currentRPM < 0.0) currentRPM = 0.0;

        lastShooterPosition = currentPosition;
        lastShooterTime = nowMs;

        // feedforward + P controller
        double ff = (targetRPM / Math.max(1.0, MAX_RPM)) * ffGain;
        double error = targetRPM - currentRPM;
        double pTerm = kP * error;
        double shooterPower = ff + pTerm;
        shooterPower = Math.max(0.0, Math.min(MAX_SHOOTER_POWER, shooterPower));

        shooter.setPower(shooterOn ? shooterPower : 0.0);

        // telemetry while running
        telemetry.addData("TargetRPM", "%.1f", targetRPM);
        telemetry.addData("RPM", "%.2f", currentRPM);
        telemetry.addData("Power", "%.3f", shooterOn ? shooter.getPower() : 0.0);
    }

    // Helper: hold for N seconds while continuously updating shooter regulation
    private void holdForSeconds(double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            updateShooter(System.currentTimeMillis());
            telemetry.update();
            idle();
        }
    }

    // New helper: wait up to maxWaitSeconds but return early if shooter reaches target ± tolerance
    private boolean waitUntilAtTargetOrTimeout(double maxWaitSeconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < maxWaitSeconds) {
            updateShooter(System.currentTimeMillis());
            if (Math.abs(targetRPM - currentRPM) <= TARGET_TOLERANCE_RPM) {
                telemetry.addData("WaitUntilAtTargetOrTimeout", "Reached after %.2fs", runtime.seconds());
                telemetry.update();
                return true;
            }
            telemetry.addData("WaitUntilAtTargetOrTimeout", "Waiting... %.2fs", runtime.seconds());
            telemetry.addData("TargetRPM", "%.1f", targetRPM);
            telemetry.addData("RPM", "%.2f", currentRPM);
            telemetry.update();
            idle();
        }
        telemetry.addData("WaitUntilAtTargetOrTimeout", "Timed out after %.2fs", maxWaitSeconds);
        telemetry.update();
        return false;
    }

    // Helper: move turret to a target encoder position (blocking until reached or timeout)
    private void moveTurretToPosition(int targetTicks, double timeoutSeconds) {
        long start = System.currentTimeMillis();
        long timeoutMs = (long) (timeoutSeconds * 1000.0);
        while (opModeIsActive()) {
            int pos = turret.getCurrentPosition();
            int error = targetTicks - pos;
            if (Math.abs(error) <= TURRET_TARGET_TOLERANCE) {
                turret.setPower(0.0);
                break;
            }
            double power = 0.25;
            if (error < 0) power = -0.25;
            if (Math.abs(error) < 50) power *= 0.5;
            turret.setPower(power);

            // keep shooter regulated while turret moves
            updateShooter(System.currentTimeMillis());
            telemetry.addData("TurretPos", pos);
            telemetry.addData("TurretTarget", targetTicks);
            telemetry.update();

            if (System.currentTimeMillis() - start > timeoutMs) {
                turret.setPower(0.0);
                break;
            }
            idle();
        }
        // ensure stop
        turret.setPower(0.0);
    }

    // Helper: set tank drive power on all four wheels
    private void setDrivePower(double leftPower, double rightPower) {
        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backRightDrive.setPower(rightPower);
    }
}