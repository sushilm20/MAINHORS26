package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="B experimentalHORS", group="Linear OpMode")
public class experimentalHORS extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, turret, intakeMotor;
    private Servo clawServo, leftCompressionServo, rightCompressionServo;
    private Servo leftHoodServo, rightHoodServo;

    // shooter control state
    private boolean shooterOn = true; // default start ON
    private static final double MAX_RPM = 200; // adjust if different
    private static final double TICKS_PER_REV = 537.6; // encoder ticks per motor rev (may require gear ratio correction)
    private double currentRPM = 0.0;       // EMA-smoothed scaled RPM used by controller
    private int lastShooterPosition = 0;
    private long lastShooterTime = 0L;

    private double targetRPM = MAX_RPM;
    private double kP = 0.0003; // tune as needed
    private double emaAlpha = 0.15; // smoother for RPM

    // RPM scaling & calibration
    private double rpmScale = 0.78; // multiply raw encoder-derived RPM by this to correct scale
    private boolean yPressedLast = false;

    // dpad/button debounce state (working for either gamepad)
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean xPressedLast = false;

    // Added: left-trigger state and saved shooter state for restore
    private boolean leftTriggerActiveLast = false;
    private double savedTargetRPMBeforeLeftTrigger = -1.0;
    private boolean savedShooterOnBeforeLeftTrigger = false;

    // rumble / in-target detection
    private boolean atTargetLast = false;
    private boolean rumbling = false;
    private long rumbleEndTimeMs = 0L;
    private static final double TARGET_TOLERANCE_RPM = 5.0;
    private static final long RUMBLE_DURATION_MS = 1000L;

    // turret limits
    private static final int TURRET_MIN_POS = -500;
    private static final int TURRET_MAX_POS = 500;

    // hood/claw timing (non-blocking)
    private double leftHoodPosition = 0.12;
    private double rightHoodPosition = 0.12;
    private long lastLeftHoodAdjustMs = 0L;
    private long lastRightHoodAdjustMs = 0L;
    private static final long HOOD_ADJUST_DEBOUNCE_MS = 120L;

    // claw action phases
    private int clawActionPhase = 0; // 0 idle, 1 closed then wait, 2 reopened
    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 500L;

    // --- NEW: Far / Close mode state controlled by gamepad1 touchpad ---
    private boolean isFarMode = false;
    private boolean touchpadPressedLast = false;

    // Mode-specific parameters (user requested values)
    private static final double RIGHT_HOOD_CLOSE   = 0.12;
    private static final double RIGHT_HOOD_FAR     = 0.24;
    private static final double TARGET_RPM_CLOSE   = 90.0;
    private static final double TARGET_RPM_FAR     = 140.0;

    @Override
    public void runOpMode() {

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

        clawServo.setPosition(0.63);
        leftCompressionServo.setPosition(0.5);
        rightCompressionServo.setPosition(0.5);
        leftHoodServo.setPosition(leftHoodPosition);

        rightHoodPosition = RIGHT_HOOD_CLOSE;
        rightHoodServo.setPosition(rightHoodPosition);

        targetRPM = TARGET_RPM_CLOSE;

        telemetry.addData("Status", "Initialized (mode = CLOSE)");
        telemetry.update();

        lastShooterPosition = shooter.getCurrentPosition();
        lastShooterTime = System.currentTimeMillis();
        shooterOn = true;

        waitForStart();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            boolean touchpadNow = false;
            try {
                touchpadNow = gamepad1.touchpad;
            } catch (Throwable t) {
                touchpadNow = (gamepad1.left_stick_button && gamepad1.right_stick_button);
            }

            if (touchpadNow && !touchpadPressedLast) {
                // Rising edge detected: toggle mode
                isFarMode = !isFarMode;

                if (isFarMode) {
                    rightHoodPosition = RIGHT_HOOD_FAR;
                    rightHoodServo.setPosition(rightHoodPosition);
                    targetRPM = TARGET_RPM_FAR;
                    shooterOn = true;

                    telemetry.addData("Mode", "FAR");
                } else {
                    rightHoodPosition = RIGHT_HOOD_CLOSE;
                    rightHoodServo.setPosition(rightHoodPosition);
                    targetRPM = TARGET_RPM_CLOSE;
                    shooterOn = true;

                    telemetry.addData("Mode", "CLOSE");
                }
            }
            touchpadPressedLast = touchpadNow;

            // -------------------------
            // DRIVE
            // -------------------------
            double axial   = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;

            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            double driveSpeed = 1.0;
            frontLeftDrive.setPower(frontLeftPower * driveSpeed);
            frontRightDrive.setPower(frontRightPower * driveSpeed);
            backLeftDrive.setPower(backLeftPower * driveSpeed);
            backRightDrive.setPower(backRightPower * driveSpeed);

            boolean dpadDownNow = gamepad1.dpad_down || gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                shooterOn = !shooterOn;
            }
            dpadDownLast = dpadDownNow;

            boolean dpadLeftNow = gamepad1.dpad_left || gamepad2.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) {
                targetRPM = Math.max(0.0, targetRPM - 10.0);
            }
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right || gamepad2.dpad_right;
            if (dpadRightNow && !dpadRightLast) {
                targetRPM = Math.min(MAX_RPM, targetRPM + 10.0);
            }
            dpadRightLast = dpadRightNow;

            // RPM measurement from encoder (non-blocking)
            int currentPosition = shooter.getCurrentPosition();
            int deltaTicks = currentPosition - lastShooterPosition;
            long deltaTimeMs = nowMs - lastShooterTime;
            if (deltaTimeMs <= 0) deltaTimeMs = 1;
            double ticksPerSec = (deltaTicks * 1000.0) / deltaTimeMs;
            double measuredRPMRaw = (ticksPerSec / TICKS_PER_REV) * 60.0; // raw (before scale)
            double measuredRPMScaled = measuredRPMRaw * rpmScale;

            // EMA smoothing
            currentRPM = (1.0 - emaAlpha) * currentRPM + emaAlpha * measuredRPMScaled;
            if (currentRPM < 0.0) currentRPM = 0.0;
            lastShooterPosition = currentPosition;
            lastShooterTime = nowMs;

            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yPressedLast) {
                double safeMeasured = Math.abs(measuredRPMRaw);
                if (safeMeasured >= 1.0) {
                    double candidateScale = targetRPM / measuredRPMRaw;
                    if (candidateScale < 0.2) candidateScale = 0.2;
                    if (candidateScale > 3.0) candidateScale = 3.0;
                    rpmScale = candidateScale;
                } else {
                    telemetry.addData("Calib", "Raw RPM too small to calibrate");
                }
            }
            yPressedLast = yNow;

            double ff = targetRPM / Math.max(1.0, MAX_RPM);
            double error = targetRPM - currentRPM;
            double pTerm = kP * error;
            double shooterPower = ff + pTerm;
            shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));
            shooter.setPower(shooterOn ? shooterPower : 0.0);

            // RUMBLE: when within tolerance for the first loop, start a 1s rumble (non-blocking)
            boolean atTargetNow = Math.abs(targetRPM - currentRPM) <= TARGET_TOLERANCE_RPM;
            if (atTargetNow && !atTargetLast) {
                rumbling = true;
                rumbleEndTimeMs = nowMs + RUMBLE_DURATION_MS;
                try { gamepad1.rumble((int) RUMBLE_DURATION_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble((int) RUMBLE_DURATION_MS); } catch (Throwable ignored) {}
            }
            atTargetLast = atTargetNow;

            if (rumbling && nowMs > rumbleEndTimeMs) {
                rumbling = false;
            }

            // ------------------------------
            // TURRET manual control using bumpers (REMOVE AprilTag tracking)
            // ------------------------------
            int turretPosition = turret.getCurrentPosition();
            double turretPower = 0.0;
            if (gamepad1.right_bumper || gamepad2.left_stick_x > 0.2) {
                if (turretPosition < TURRET_MAX_POS) {
                    turretPower = 0.2;
                }
            } else if (gamepad1.left_bumper || gamepad2.left_stick_x < -0.2) {
                if (turretPosition > TURRET_MIN_POS) {
                    turretPower = -0.2;
                }
            } else {
                turretPower = 0.0;
            }
            turret.setPower(turretPower);

            // INTAKE + COMPRESSION
            boolean leftTriggerNow = gamepad1.left_trigger > 0.1;

            if (leftTriggerNow) {
                if (!leftTriggerActiveLast) {
                    savedTargetRPMBeforeLeftTrigger = targetRPM;
                    savedShooterOnBeforeLeftTrigger = shooterOn;
                    targetRPM = 40.0;
                    shooterOn = true;
                }
                intakeMotor.setPower(-1.0);
                leftCompressionServo.setPosition(0.0);
                rightCompressionServo.setPosition(1.0);
            } else {
                if (leftTriggerActiveLast) {
                    if (savedTargetRPMBeforeLeftTrigger >= 0.0) {
                        targetRPM = savedTargetRPMBeforeLeftTrigger;
                        savedTargetRPMBeforeLeftTrigger = -1.0;
                    }
                    shooterOn = savedShooterOnBeforeLeftTrigger;
                    savedShooterOnBeforeLeftTrigger = false;
                }

                if ((gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                    intakeMotor.setPower(1.0);
                    leftCompressionServo.setPosition(1.0);
                    rightCompressionServo.setPosition(0.0);
                } else {
                    intakeMotor.setPower(0.0);
                    leftCompressionServo.setPosition(0.5);
                    rightCompressionServo.setPosition(0.5);
                }
            }

            leftTriggerActiveLast = leftTriggerNow;

            // CLAW single-press toggle using non-blocking timed phases
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xPressedLast) {
                clawServo.setPosition(0.2);
                clawActionPhase = 1;
                clawActionStartMs = nowMs;
            }
            xPressedLast = xNow;

            if (clawActionPhase == 1 && nowMs >= clawActionStartMs + CLAW_CLOSE_MS) {
                clawServo.setPosition(0.63);
                clawActionPhase = 0;
            }

            // LEFT HOOD servo adjustments with A/B (gamepad1) rate-limited (non-blocking)
            if (gamepad1.a && nowMs - lastLeftHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastLeftHoodAdjustMs = nowMs;
                leftHoodPosition += 0.025;
                if (leftHoodPosition > 0.45) leftHoodPosition = 0.45;
                leftHoodServo.setPosition(leftHoodPosition);
            }
            if (gamepad1.b && nowMs - lastLeftHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastLeftHoodAdjustMs = nowMs;
                leftHoodPosition -= 0.025;
                if (leftHoodPosition < 0.12) leftHoodPosition = 0.12;
                leftHoodServo.setPosition(leftHoodPosition);
            }

            // RIGHT HOOD servo controlled by gamepad2 right stick Y, with rate-limited adjustments
            if (gamepad2.right_stick_y < -0.2 && nowMs - lastRightHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastRightHoodAdjustMs = nowMs;
                rightHoodPosition += 0.01;
                if (rightHoodPosition > 0.45) rightHoodPosition = 0.45;
                rightHoodServo.setPosition(rightHoodPosition);
            } else if (gamepad2.right_stick_y > 0.2 && nowMs - lastRightHoodAdjustMs >= HOOD_ADJUST_DEBOUNCE_MS) {
                lastRightHoodAdjustMs = nowMs;
                rightHoodPosition -= 0.01;
                if (rightHoodPosition < 0.12) rightHoodPosition = 0.12;
                rightHoodServo.setPosition(rightHoodPosition);
            }

            telemetry.addData("Mode", isFarMode ? "FAR" : "CLOSE");
            telemetry.addData("Shooter State", shooterOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Measured RPM Raw", "%.2f", measuredRPMRaw);
            telemetry.addData("Measured RPM Scaled", "%.2f", measuredRPMScaled);
            telemetry.addData("Smoothed RPM (used)", "%.2f", currentRPM);
            telemetry.addData("rpmScale", "%.4f", rpmScale);
            telemetry.addData("Shooter Power", "%.3f", shooterOn ? shooterPower : 0.0);
            telemetry.addData("Turret Encoder", turret.getCurrentPosition());
            telemetry.update();
        }

    }
}