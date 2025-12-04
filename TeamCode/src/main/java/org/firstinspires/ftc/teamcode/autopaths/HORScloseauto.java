package org.firstinspires.ftc.teamcode.autopaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="HORS Close Blue", group="Linear OpMode")
public class HORScloseauto extends LinearOpMode {

    // Drive motors (same names as teleop)
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    // Mechanism motors/servos (same as teleop)
    private DcMotor shooter, turret, intakeMotor;
    private Servo clawServo, leftCompressionServo, rightCompressionServo;
    private Servo leftHoodServo, rightHoodServo;

    private ElapsedTime runtime = new ElapsedTime();

    // Shooter control state & tuning (copied from your teleop)
    private boolean shooterOn = false; // start OFF during movement stage per request
    private static final double MAX_RPM = 200.0; // matches your teleop file. the gobilda only does 190, (this is not rpm output of motor but the ticks per sec)
    private static final double TICKS_PER_REV = 537.6;
    private double currentRPM = 0.0;
    private int lastShooterPosition = 0;
    private long lastShooterTime = 0L;

    private double targetRPM = 120.0; // 120 rpm shooting
    private double kP = 0.0003;
    private double emaAlpha = 0.15;

    // Scaling and calibration (kept from your teleop)
    private double rpmScale = 0.78; // this should be same from ur teleoppping.
    private double ffGain = 0.8;
    private static final double MAX_SHOOTER_POWER = 0.9;

    // Hood/claw positions
    private double leftHoodPosition = 0.12;
    private double rightHoodPosition = 0.12;

    // Claw timing
    private static final long CLAW_CLOSE_MS = 500L;

    // Turret movement tolerance
    private static final int TURRET_TARGET_TOLERANCE = 5;

    @Override
    public void runOpMode() {
        // Hardware map (same names used in teleop)
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

        // Directions and modes (match teleop)
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

        // Initial servo positions
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

        waitForStart();

        // Sequence requested:
        // 1) drive forward 1s
        // 2) turn right 1s
        // After movement/turn completed:
        // 3) turn shooter on at RPM=120 (use PID loop)
        // 4) rotate turret to encoder position 300
        // 5) right hood servo to 0.24
        // 6) intake sequence: spin 1s, stop 1s, spin 1s, spin 1s, then claw action
        // 7) turn off shooter and then move sideways (strafe) at the end
        // 8) stop everything and finish

        // 1) Drive forward for 1s
        setDrivePower(-0.6, -0.6);
        holdForSeconds(1.0);

        // brief stop
        setDrivePower(0.0, 0.0);
        holdForSeconds(0.05);

//        // 2) Turn right for 1s (left positive, right negative)
//        setDrivePower(0.5, -0.5);
//        holdForSeconds(1.165);//gamble this

        // stop drive
        setDrivePower(0.0, 0.0);
        holdForSeconds(0.05);

        // 3) Turn shooter ON at 120 RPM (shooter loop will run continuously)
        targetRPM = 135;
        shooterOn = true;
        // kick-off shooter regulation loop briefly before turret movement
        // hold a short period so shooter can begin spinning while turret moves
        holdForSeconds(3);

        // 4) Rotate turret to encoder position 300 (blocking until reached or timeout)
        moveTurretToPosition(-1, 3); // 3s timeout to avoid infinite loop

        // 5) Adjust shooter angle: right hood to 0.24 (shooter continues to be regulated)
        rightHoodPosition = 0.20;
        rightHoodServo.setPosition(rightHoodPosition);
        holdForSeconds(0.15);

        // 6) Intake automatic sequence:
        // Sequence: spin 1s, pause 1s, spin 1s, spin 1s, then claw action
        // Move #1: intake ON 1s
        leftCompressionServo.setPosition(1.0);
        rightCompressionServo.setPosition(0.0);
        intakeMotor.setPower(1.0);
        holdForSeconds(1.0);

        // Pause 1s
        intakeMotor.setPower(0.0);
        leftCompressionServo.setPosition(0.5);
        rightCompressionServo.setPosition(0.5);
        holdForSeconds(1.0);

        // Move #2: intake ON 1s
        leftCompressionServo.setPosition(1.0);
        rightCompressionServo.setPosition(0.0);
        intakeMotor.setPower(1.03);
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

        // Claw action: close then reopen (simulates X button action)
        clawServo.setPosition(0.2); // close
        holdForSeconds(CLAW_CLOSE_MS / 1000.0);
        clawServo.setPosition(0.63); // reopen

        // Short settle while shooter still regulated
        holdForSeconds(0.1);

        // 7) Turn off shooter motor and stop mechanisms (but do NOT stop drive yet)
        shooterOn = false;
        shooter.setPower(0.0);

        intakeMotor.setPower(0.0);
        turret.setPower(0.0);

        // 8) Move sideways at the end: strafe right for 1s
        setStrafePower(0.6);
        holdForSeconds(1.0);

        // stop drive after strafing
        setDrivePower(0.0, 0.0);

        telemetry.addData("Auto", "Sequence complete (including end strafe)");
        telemetry.update();

        // final brief hold so telemetry can be read
        holdForSeconds(0.5);
    }

    // Helper: update shooter regulation using same logic as teleop
    private void updateShooter(long nowMs) {
        // read encoder and compute RPM
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

        // feedforward + P controller (same math as teleop)
        double ff = (targetRPM / Math.max(1.0, MAX_RPM)) * ffGain;
        double error = targetRPM - currentRPM;
        double pTerm = kP * error;
        double shooterPower = ff + pTerm;
        shooterPower = Math.max(0.0, Math.min(MAX_SHOOTER_POWER, shooterPower));

        shooter.setPower(shooterOn ? shooterPower : 0.0);

        // telemetry for debugging
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

    // Helper: move turret to a target encoder position (blocking until reached or timeout)
    private void moveTurretToPosition(int targetTicks, double timeoutSeconds) {
        long start = System.currentTimeMillis();
        long timeoutMs = (long) (timeoutSeconds * 1000.0);
        // simple proportional-ish motion: small constant power until near target
        while (opModeIsActive()) {
            int pos = turret.getCurrentPosition();
            int error = targetTicks - pos;
            if (Math.abs(error) <= TURRET_TARGET_TOLERANCE) {
                turret.setPower(0.0);
                break;
            }
            // decide direction
            double power = 0.25;
            if (error < 0) power = -0.25;
            // reduce power when close
            if (Math.abs(error) < 50) power *= 0.5;
            turret.setPower(power);

            // keep shooter regulated while turret moves
            updateShooter(System.currentTimeMillis());
            telemetry.addData("TurretPos", pos);
            telemetry.addData("TurretTarget", targetTicks);
            telemetry.update();

            if (System.currentTimeMillis() - start > timeoutMs) {
                // timeout reached
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

    // Helper: set mecanum strafe power (positive -> strafe right)
    private void setStrafePower(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(-power);
        frontRightDrive.setPower(-power);
        backRightDrive.setPower(power);
    }
}