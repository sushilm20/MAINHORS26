package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//testing wild things
@TeleOp(name="???HORS???", group="Linear OpMode")
public class secondexperimentalHORS extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, turret, intakeMotor;
    private Servo clawServo, leftCompressionServo, rightCompressionServo;
    private Servo leftHoodServo, rightHoodServo;

    // shooter control state (unchanged)
    private boolean shooterOn = true;
    private static final double MAX_RPM = 200;
    private static final double TICKS_PER_REV = 537.6;
    private double currentRPM = 0.0;
    private int lastShooterPosition = 0;
    private long lastShooterTime = 0L;

    private double targetRPM = MAX_RPM;
    private double kP = 0.0003;
    private double emaAlpha = 0.15;
    private double rpmScale = 0.78;
    private boolean yPressedLast = false;

    // UI / debounce
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean xPressedLast = false;

    private boolean leftTriggerActiveLast = false;
    private double savedTargetRPMBeforeLeftTrigger = -1.0;
    private boolean savedShooterOnBeforeLeftTrigger = false;

    private boolean atTargetLast = false;
    private boolean rumbling = false;
    private long rumbleEndTimeMs = 0L;
    private static final double TARGET_TOLERANCE_RPM = 5.0;
    private static final long RUMBLE_DURATION_MS = 1000L;

    // turret hard limits
    private static final int TURRET_MIN_POS = -500;
    private static final int TURRET_MAX_POS = 500;

    // hood/claw
    private double leftHoodPosition = 0.12;
    private double rightHoodPosition = 0.12;
    private long lastLeftHoodAdjustMs = 0L;
    private long lastRightHoodAdjustMs = 0L;
    private static final long HOOD_ADJUST_DEBOUNCE_MS = 120L;

    private int clawActionPhase = 0;
    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 500L;

    // Far/Close mode
    private boolean isFarMode = false;
    private boolean touchpadPressedLast = false;
    private static final double RIGHT_HOOD_CLOSE   = 0.12;
    private static final double RIGHT_HOOD_FAR     = 0.24;
    private static final double TARGET_RPM_CLOSE   = 90.0;
    private static final double TARGET_RPM_FAR     = 140.0;

    // IMU & heading references
    private BNO055IMU imu;
    private double headingReferenceRad = 0.0;
    private double lastHeadingRad = 0.0;     // for angular velocity calc
    private int turretEncoderReference = 0;

    // ticks-per-radian mapping
    private static final double BASE_TICKS_PER_RADIAN = TURRET_MAX_POS / Math.PI; // ~159.155
    private static final double TICKS_PER_RADIAN_SCALE = 2.88;
    private static final double TICKS_PER_RADIAN = BASE_TICKS_PER_RADIAN * TICKS_PER_RADIAN_SCALE;

    // PID parameters tuned for damping and stability (reduced aggressiveness)
    private static final double TURRET_KP = 0.033;
    private static final double TURRET_KI = 0.0000;
    private static final double TURRET_KD = 0.010;
    private static final double TURRET_MAX_POWER = 0.80;

    // feedforward gain (based on robot angular velocity in rad/s)
    private static final double FF_GAIN = 0.03;

    // deadband and anti-windup
    private static final int SMALL_DEADBAND_TICKS = 3;
    private static final double INTEGRAL_CLAMP = 200.0;

    // heavy smoothing to remove violent shaking
    private static final double POWER_SMOOTH_ALPHA = 0.96;

    // derivative low-pass filter to avoid spike noises
    private static final double DERIV_FILTER_ALPHA = 0.2;

    // PID state
    private double turretIntegral = 0.0;
    private int lastTurretError = 0;
    private long lastTurretTimeMs = 0L;
    private double lastAppliedTurretPower = 0.0;
    private double lastDerivative = 0.0;

    // manual override tracking
    private boolean manualTurretActiveLast = false;

    // For gamepad2 touchpad reset
    private boolean gamepad2TouchpadLast = false;

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

        // keep turret reversed as requested
        turret.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU init
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(imuParams);

        // initial positions
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

        // capture initial references
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;
        turretEncoderReference = turret.getCurrentPosition();
        lastTurretTimeMs = System.currentTimeMillis();

        waitForStart();

        // reset state at start
        headingReferenceRad = getHeadingRadians();
        lastHeadingRad = headingReferenceRad;
        turretEncoderReference = turret.getCurrentPosition();
        lastTurretTimeMs = System.currentTimeMillis();
        manualTurretActiveLast = false;
        turretIntegral = 0.0;
        lastTurretError = 0;
        lastAppliedTurretPower = 0.0;
        lastDerivative = 0.0;

        gamepad2TouchpadLast = false;

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            boolean touchpadNow = false;
            try {
                touchpadNow = gamepad1.touchpad;
            } catch (Throwable t) {
                touchpadNow = (gamepad1.left_stick_button && gamepad1.right_stick_button);
            }

            // ------------------------------------
            // NEW FEATURE: gamepad2 touchpad reset
            // ------------------------------------
            boolean gamepad2TouchpadNow = false;
            try {
                gamepad2TouchpadNow = gamepad2.touchpad;
            } catch (Throwable t) {
                // fallback: both sticks pressed for gamepad2 (legacy)
                gamepad2TouchpadNow = (gamepad2.left_stick_button && gamepad2.right_stick_button);
            }
            if (gamepad2TouchpadNow && !gamepad2TouchpadLast) {
                // Reset IMU Z heading reference (no actual IMU re-init, just software reference)
                headingReferenceRad = 0.0;
                lastHeadingRad = 0.0;

                // Reset turret encoder
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                turretEncoderReference = 0;
                lastTurretError = 0;
                turretIntegral = 0.0;
                lastAppliedTurretPower = 0.0;
                lastDerivative = 0.0;

                telemetry.addData("Reset", "IMU heading reference and turret encoder set to zero!");
                telemetry.update();
            }
            gamepad2TouchpadLast = gamepad2TouchpadNow;

            if (touchpadNow && !touchpadPressedLast) {
                isFarMode = !isFarMode;
                if (isFarMode) {
                    rightHoodPosition = RIGHT_HOOD_FAR;
                    rightHoodServo.setPosition(rightHoodPosition);
                    targetRPM = TARGET_RPM_FAR;
                    shooterOn = true;
                } else {
                    rightHoodPosition = RIGHT_HOOD_CLOSE;
                    rightHoodServo.setPosition(rightHoodPosition);
                    targetRPM = TARGET_RPM_CLOSE;
                    shooterOn = true;
                }
            }
            touchpadPressedLast = touchpadNow;

            // DRIVE (unchanged)
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

            // dpad toggles
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

            // RPM measurement from encoder (unchanged)
            int shooterCurrentPosition = shooter.getCurrentPosition();
            int deltaTicks = shooterCurrentPosition - lastShooterPosition;
            long deltaTimeMs = nowMs - lastShooterTime;
            if (deltaTimeMs <= 0) deltaTimeMs = 1;
            double ticksPerSec = (deltaTicks * 1000.0) / deltaTimeMs;
            double measuredRPMRaw = (ticksPerSec / TICKS_PER_REV) * 60.0;
            double measuredRPMScaled = measuredRPMRaw * rpmScale;

            currentRPM = (1.0 - emaAlpha) * currentRPM + emaAlpha * measuredRPMScaled;
            if (currentRPM < 0.0) currentRPM = 0.0;
            lastShooterPosition = shooterCurrentPosition;
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

            // rumble handling
            boolean atTargetNow = Math.abs(targetRPM - currentRPM) <= TARGET_TOLERANCE_RPM;
            if (atTargetNow && !atTargetLast) {
                rumbling = true;
                rumbleEndTimeMs = nowMs + RUMBLE_DURATION_MS;
                try { gamepad1.rumble((int) RUMBLE_DURATION_MS); } catch (Throwable ignored) {}
                try { gamepad2.rumble((int) RUMBLE_DURATION_MS); } catch (Throwable ignored) {}
            }
            atTargetLast = atTargetNow;
            if (rumbling && nowMs > rumbleEndTimeMs) rumbling = false;

            // TURRET: SMOOTH PID + IMU FEEDFORWARD
            int turretPos = turret.getCurrentPosition();

            boolean manualNow = false;
            double manualPower = 0.0;

            // manual control: bumpers OR legacy stick
            if (gamepad1.right_bumper || gamepad2.left_stick_x > 0.2) {
                manualNow = true;
                if (turretPos < TURRET_MAX_POS) manualPower = 0.5;
            } else if (gamepad1.left_bumper || gamepad2.left_stick_x < -0.2) {
                manualNow = true;
                if (turretPos > TURRET_MIN_POS) manualPower = -0.5;
            }

            if (manualNow) {
                // enforce limits
                if (turretPos >= TURRET_MAX_POS && manualPower > 0.0) manualPower = 0.0;
                if (turretPos <= TURRET_MIN_POS && manualPower < 0.0) manualPower = 0.0;
                turret.setPower(manualPower);

                // reset PID state
                turretIntegral = 0.0;
                lastTurretError = 0;
                lastTurretTimeMs = nowMs;
                lastAppliedTurretPower = 0.0;
                lastDerivative = 0.0;
            } else {
                // on transition from manual to auto, re-capture references and reset PID state
                if (manualTurretActiveLast) {
                    headingReferenceRad = getHeadingRadians();
                    lastHeadingRad = headingReferenceRad;
                    turretEncoderReference = turret.getCurrentPosition();
                    turretIntegral = 0.0;
                    lastTurretError = 0;
                    lastTurretTimeMs = nowMs;
                    lastAppliedTurretPower = 0.0;
                    lastDerivative = 0.0;
                }

                // heading & angular velocity
                double currentHeadingRad = getHeadingRadians();
                long dtMsHeading = nowMs - lastTurretTimeMs;
                if (dtMsHeading <= 0) dtMsHeading = 1;
                double dtHeading = dtMsHeading / 1000.0;

                // angular velocity estimated from heading difference
                double headingDeltaSinceLast = normalizeAngle(currentHeadingRad - lastHeadingRad);
                double angularVel = headingDeltaSinceLast / Math.max(0.0001, dtHeading); // rad/s

                lastHeadingRad = currentHeadingRad;

                // desired encoder ticks from heading delta relative to reference
                double deltaHeading = normalizeAngle(currentHeadingRad - headingReferenceRad);
                double desiredTicksDouble = turretEncoderReference - deltaHeading * TICKS_PER_RADIAN;
                int desiredTicks = (int) Math.round(desiredTicksDouble);

                // clamp desired ticks
                if (desiredTicks > TURRET_MAX_POS) desiredTicks = TURRET_MAX_POS;
                if (desiredTicks < TURRET_MIN_POS) desiredTicks = TURRET_MIN_POS;

                int errorTicks = desiredTicks - turretPos;

                // PID timing
                long dtMs = nowMs - lastTurretTimeMs;
                if (dtMs <= 0) dtMs = 1;
                double dt = dtMs / 1000.0;

                // integrate with anti-windup
                turretIntegral += errorTicks * dt;
                if (turretIntegral > INTEGRAL_CLAMP) turretIntegral = INTEGRAL_CLAMP;
                if (turretIntegral < -INTEGRAL_CLAMP) turretIntegral = -INTEGRAL_CLAMP;

                // derivative and filter
                double rawDerivative = (errorTicks - lastTurretError) / Math.max(0.0001, dt);
                double derivativeFiltered = DERIV_FILTER_ALPHA * rawDerivative + (1.0 - DERIV_FILTER_ALPHA) * lastDerivative;

                // PID output
                double pidOut = TURRET_KP * errorTicks + TURRET_KI * turretIntegral + TURRET_KD * derivativeFiltered;

                // IMU feedforward based on angular velocity: turret should move opposite robot angular velocity
                ff = -angularVel * FF_GAIN;

                double cmdPower = pidOut + ff;

                // deadband: keep it slightly larger to avoid tiny corrections causing jitter
                if (Math.abs(errorTicks) <= SMALL_DEADBAND_TICKS) {
                    cmdPower = 0.0;
                }

                // clamp to max allowed motor power
                if (cmdPower > TURRET_MAX_POWER) cmdPower = TURRET_MAX_POWER;
                if (cmdPower < -TURRET_MAX_POWER) cmdPower = -TURRET_MAX_POWER;

                // strong smoothing to prevent violent changes
                double applied = POWER_SMOOTH_ALPHA * lastAppliedTurretPower + (1.0 - POWER_SMOOTH_ALPHA) * cmdPower;

                // if at hard stop, block commands that would push further
                if ((turretPos >= TURRET_MAX_POS && applied > 0.0) || (turretPos <= TURRET_MIN_POS && applied < 0.0)) {
                    applied = 0.0;
                }

                turret.setPower(applied);

                // update PID history
                lastTurretError = errorTicks;
                lastTurretTimeMs = nowMs;
                lastAppliedTurretPower = applied;
                lastDerivative = derivativeFiltered;

                telemetry.addData("turret.desired", desiredTicks);
                telemetry.addData("turret.error", errorTicks);
                telemetry.addData("turret.pid", String.format("%.3f", pidOut));
                telemetry.addData("turret.ff", String.format("%.3f", ff));
                telemetry.addData("turret.applied", String.format("%.3f", applied));
            }

            manualTurretActiveLast = manualNow;

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

            // CLAW toggle
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

            // Hood adjustments
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

            // summary telemetry
            telemetry.addData("Mode", isFarMode ? "FAR" : "CLOSE");
            telemetry.addData("Turret Enc", turret.getCurrentPosition());
            telemetry.addData("Turret Power", lastAppliedTurretPower);
            telemetry.update();
        }
    }

    /**
     * Read IMU heading (Z axis) in radians in range (-pi, pi]
     * Correction for Rev Hub: logo RIGHT, USB DOWN (axes mirrored from default).
     */
    private double getHeadingRadians() {
        if (imu == null) return 0.0;
        Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -o.firstAngle;
    }

    /**
     * Normalize angle to range (-pi, pi]
     */
    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }
}