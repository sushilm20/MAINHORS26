package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="z Inverted", group="Linear OpMode")
public class invertedHORS extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    private DcMotor shooter, turret, intakeMotor;
    private Servo clawServo, leftCompressionServo, rightCompressionServo;
    private Servo leftHoodServo, rightHoodServo;

    private boolean shooterOn = false;
    private boolean dpadDownPressedLast = false;
    private boolean dpadLeftPressedLast = false;
    private boolean dpadRightPressedLast = false;
    private double hoodServoPosition = 0.12;

    // For claw single-press toggle
    private boolean xPressedLast = false;

    private static final double TICKS_PER_REV = 537.6;
    private double shooterRPM = 0.0;
    private int lastShooterPosition = 0;
    private long lastShooterTime = 0;

    private double targetRPM = 6000.0; // Start at max
    private double kP = 0.0003;

    // Turret min and max encoder ticks
    private static final int TURRET_MIN_POS = -500;
    private static final int TURRET_MAX_POS = 500;

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
        leftHoodServo.setPosition(hoodServoPosition);
        rightHoodServo.setPosition(hoodServoPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        lastShooterPosition = shooter.getCurrentPosition();
        lastShooterTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            double axial   = -gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;
            double rightStickY = gamepad2.right_stick_y;

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




            // Shooter toggle with dpad
            if ((gamepad1.dpad_down || gamepad2.dpad_down) && !dpadDownPressedLast) {
                shooterOn = !shooterOn;
                dpadDownPressedLast = true;
            } else if (!gamepad1.dpad_down && !gamepad2.dpad_down) {
                dpadDownPressedLast = false;
            }




            // RPM adjustment: reduce by 10 with dpad left, increase by 10 with dpad right
            if (gamepad1.dpad_left && !dpadLeftPressedLast) {
                targetRPM = Math.max(targetRPM - 10, 0);
                dpadLeftPressedLast = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressedLast = false;
            }



            if (gamepad1.dpad_right && !dpadRightPressedLast) {
                targetRPM = targetRPM + 10;
                dpadRightPressedLast = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressedLast = false;
            }

            // ---- IMPROVED Shooter RPM calculation ----
            int currentPosition = shooter.getCurrentPosition();
            long currentTime = System.currentTimeMillis();

            int deltaTicks = currentPosition - lastShooterPosition;
            long deltaTime = currentTime - lastShooterTime;

            if (deltaTime > 0) {
                double ticksPerSec = (deltaTicks * 1000.0) / deltaTime;
                shooterRPM = (ticksPerSec / TICKS_PER_REV) * 60.0;
            }

            lastShooterPosition = currentPosition;
            lastShooterTime = currentTime;

            // Shooter power control
            double shooterPower = kP * (targetRPM - shooterRPM);
            shooterPower = Math.max(0.0, Math.min(shooterPower, 1.0));
            shooter.setPower(shooterOn ? shooterPower : 0.0);

            // Turret control: gamepad1 bumpers OR gamepad2 left stick with encoder limits
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

            // Intake + compression
            if ((gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                intakeMotor.setPower(1.0);
                leftCompressionServo.setPosition(1.0);
                rightCompressionServo.setPosition(0.0);
            } else {
                intakeMotor.setPower(0.0);
                leftCompressionServo.setPosition(0.5);
                rightCompressionServo.setPosition(0.5);
            }

            // Claw toggle (single press for either gamepad)
            if ((gamepad1.x || gamepad2.x) && !xPressedLast) {
                clawServo.setPosition(0.2);
                sleep(500);
                clawServo.setPosition(0.63);
                xPressedLast = true;
            } else if (!(gamepad1.x || gamepad2.x)) {
                xPressedLast = false;
            }

            // GAMEPAD 1: A/B adjust left hood servo position
            if (gamepad1.a && hoodServoPosition < 0.45) {
                hoodServoPosition += 0.025;
                if (hoodServoPosition > 0.45) hoodServoPosition = 0.45;
                sleep(120);
            }
            if (gamepad1.b && hoodServoPosition > 0.12) {
                hoodServoPosition -= 0.025;
                if (hoodServoPosition < 0.12) hoodServoPosition = 0.12;
                sleep(120);
            }
            leftHoodServo.setPosition(hoodServoPosition);

            // GAMEPAD 2: Right stick Y controls right hood servo
            if (gamepad2.right_stick_y < -0.2 && rightHoodServo.getPosition() < 0.45) {
                double newPos = rightHoodServo.getPosition() + 0.01;
                rightHoodServo.setPosition(Math.min(newPos, 0.45));
                sleep(100);
            } else if (gamepad2.right_stick_y > 0.2 && rightHoodServo.getPosition() > 0.12) {
                double newPos = rightHoodServo.getPosition() - 0.01;
                rightHoodServo.setPosition(Math.max(newPos, 0.12));
                sleep(100);
            }

            frontLeftDrive.setPower(frontLeftPower * driveSpeed);
            frontRightDrive.setPower(frontRightPower * driveSpeed);
            backLeftDrive.setPower(backLeftPower * driveSpeed);
            backRightDrive.setPower(backRightPower * driveSpeed);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Shooter RPM", "%4.2f", shooterOn ? shooterRPM : 0.0);
            telemetry.addData("Target RPM", "%4.0f", targetRPM);
            telemetry.addData("Shooter State", shooterOn ? "ON" : "OFF");
            telemetry.addData("Turret Encoder", turret.getCurrentPosition());
            telemetry.addData("Claw", "%4.2f", clawServo.getPosition());
            telemetry.addData("Left Compression", "%4.2f", leftCompressionServo.getPosition());
            telemetry.addData("Right Compression", "%4.2f", rightCompressionServo.getPosition());
            telemetry.addData("Hood Servo", "%4.2f", hoodServoPosition);
            telemetry.update();
        }
    }
}