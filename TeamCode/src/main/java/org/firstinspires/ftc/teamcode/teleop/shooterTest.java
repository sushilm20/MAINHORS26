package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Shooter Test (full power)", group = "Test")
public class shooterTest extends LinearOpMode {

    private DcMotor shooter;
    private DcMotor shooter2;

    @Override
    public void runOpMode() {

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        // Set directions if needed to get opposite spin; adjust to match your robot
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);

        // Use raw power control (no encoder)
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready - will run both shooters at full power on start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Full power
        shooter.setPower(1.0);
        shooter2.setPower(1.0);

        telemetry.addData("Shooter", "Both motors running at full power");
        telemetry.update();

        // Hold until stop
        while (opModeIsActive()) {
            sleep(50);
        }

        // Stop on exit
        shooter.setPower(0.0);
        shooter2.setPower(0.0);
    }
}