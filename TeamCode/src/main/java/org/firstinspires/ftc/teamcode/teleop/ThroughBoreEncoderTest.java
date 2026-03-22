package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "Through Bore Encoder Test", group = "Test")
public class ThroughBoreEncoderTest extends LinearOpMode {

    DcMotor encoder;

    @Override
    public void runOpMode() {

        encoder = hardwareMap.get(DcMotor.class, "turret");

        // Reset encoder to zero
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run without motor power but still read encoder ticks
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Encoder Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int ticks = encoder.getCurrentPosition();

            telemetry.addData("Encoder Ticks", ticks);
            telemetry.update();
        }
    }
}
