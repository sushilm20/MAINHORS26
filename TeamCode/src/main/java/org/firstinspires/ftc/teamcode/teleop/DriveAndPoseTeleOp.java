package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="DriveAndPoseTeleOp", group="TeleOp")
public class DriveAndPoseTeleOp extends OpMode {

    private Follower follower;      // PedroPathing follower used for pose tracking and driving
    private Pose currentPose = new Pose();  // Robot pose, updated each loop

    @Override
    public void init() {
        try {
            // Initialize PedroPathing follower for pose tracking and drivetrain control
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));  // Default starting pose (adjust as needed)
        } catch (Exception e) {
            telemetry.addData("Error", "Follower initialization failed!");
            follower = null;
        }
    }

    @Override
    public void start() {
        if (follower != null) {
            follower.startTeleopDrive();
        }
    }

    @Override
    public void loop() {
        // Update follower (fetch or compute pose values)
        if (follower != null) {
            follower.update();  // Update pose and drivetrain
            currentPose = follower.getPose();  // Retrieve current robot pose
        }

        // Drive controls (robot-centric driving)
        if (follower != null) {
            double axial = -gamepad1.left_stick_y;     // Forward/backward control
            double lateral = -gamepad1.left_stick_x;  // Left/right control
            double yaw = -gamepad1.right_stick_x;     // Rotation control
            follower.setTeleOpDrive(axial, lateral, yaw, true);  // Robot-centric driving enabled
        }

        // Telemetry: display robot pose, accurate to the tenths place
        telemetry.addData("Pose", currentPose != null
                ? String.format("(%.1f, %.1f, %.1f°)", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()))
                : "N/A");
        telemetry.update();
    }
}
