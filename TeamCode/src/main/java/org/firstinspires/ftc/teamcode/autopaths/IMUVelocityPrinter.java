package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Simple diagnostic OpMode that continuously prints the IMU velocity (x, y, z).
 *
 * This uses a continuous loop (while opModeIsActive()) to repeatedly read imu.getVelocity()
 * and print x/y/z fields to Panels telemetry and driver-station telemetry.
 *
 * Note: different IMU firmware / SDK versions expose different field names; this code attempts
 * to read the commonly used BNO055IMU.Acceleration fields (xAccel/yAccel/zAccel). If those
 * are not present in your SDK, the code will fall back to printing the raw toString() value.
 */
@Autonomous(name = "IMU Velocity Printer", group = "Diagnostics")
public class IMUVelocityPrinter extends LinearOpMode {

//    private Telemetry panelsTelemetry;
    private GoBildaPinpointDriver imu;

    @Override
    public void runOpMode() {

//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();

        // Try to map and initialize the IMU
        try {
            imu = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            imu.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );
            imu.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//            BNO055IMU.Parameters );
        } catch (Exception e) {
//            panelsTelemetry.addData("Init", "IMU mapping/init failed: " + e.getMessage());
//            panelsTelemetry.update();
            // cannot proceed sensibly if IMU isn't present, but still wait for start so user can see error
        }

        telemetry.addData("Status", "Ready - press PLAY to start printing IMU velocity");
        telemetry.update();
        waitForStart();

        // Continuous loop: read imu.getVelocity() and print x/y/z until opmode stops.
        while (opModeIsActive()) {
            telemetry.addData("VelX", String.format("%.4f", imu.getVelX(DistanceUnit.CM)));
            telemetry.addData("VelY", String.format("%.4f", imu.getVelY(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}