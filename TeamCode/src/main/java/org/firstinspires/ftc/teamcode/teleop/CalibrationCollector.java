package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Calibration Collector 📊", group="Tuning")
public class CalibrationCollector extends LinearOpMode {

    // Your goal position
    private static final Pose GOAL = new Pose(0, 144, 0);

    private Follower follower;
    private DriveController driveController;
    private FlywheelController flywheel;

    // Panels telemetry
    private TelemetryManager panelsTelemetry;

    private List<String> collectedPoints = new ArrayList<>();
    private double currentRpm = 2400;

    private boolean aPressedLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean bPressedLast = false;
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;

    @Override
    public void runOpMode() {
        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize hardware
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        // Set directions (match your wildexperiment settings)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);

        // Initialize controllers
        driveController = new DriveController(frontLeft, frontRight, backLeft, backRight);
        flywheel = new FlywheelController(shooter, shooter2, telemetry, null);
        flywheel.setShooterOn(true);
        flywheel.setTargetRPM(currentRpm);

        // Initialize follower for pose tracking
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(20, 122, Math.toRadians(130)));
        } catch (Exception e) {
            follower = null;
        }

        telemetry.addLine("=== CALIBRATION COLLECTOR ===");
        telemetry.addLine("LEFT STICK = Drive");
        telemetry.addLine("RIGHT STICK = Turn");
        telemetry.addLine("DPAD LEFT/RIGHT = Adjust RPM ±50");
        telemetry.addLine("DPAD UP/DOWN = Adjust RPM ±10");
        telemetry.addLine("A = Save calibration point");
        telemetry.addLine("Y = Print all points");
        telemetry.addLine("B = Toggle shooter on/off");
        telemetry.update();

        // Push init status to Panels
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Points", 0);
        panelsTelemetry.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // Update pose tracking
            if (follower != null) {
                follower.update();
            }
            Pose currentPose = (follower != null) ? follower.getPose() : new Pose(0, 0, 0);
            double distance = distanceToGoal(currentPose);

            // DRIVE (left stick = move, right stick = turn)
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            driveController.setDrive(axial, lateral, yaw, 0.7);  // 70% speed for precision

            // ADJUST RPM ±50 (DPAD LEFT/RIGHT)
            boolean dpadLeftNow = gamepad1.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) {
                currentRpm -= 50;
                flywheel.setTargetRPM(currentRpm);
            }
            dpadLeftLast = dpadLeftNow;

            boolean dpadRightNow = gamepad1.dpad_right;
            if (dpadRightNow && !dpadRightLast) {
                currentRpm += 50;
                flywheel.setTargetRPM(currentRpm);
            }
            dpadRightLast = dpadRightNow;

            // ADJUST RPM ±10 (DPAD UP/DOWN) - for fine tuning
            boolean dpadUpNow = gamepad1.dpad_up;
            if (dpadUpNow && !dpadUpLast) {
                currentRpm += 10;
                flywheel.setTargetRPM(currentRpm);
            }
            dpadUpLast = dpadUpNow;

            boolean dpadDownNow = gamepad1.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                currentRpm -= 10;
                flywheel.setTargetRPM(currentRpm);
            }
            dpadDownLast = dpadDownNow;

            // TOGGLE SHOOTER (B)
            boolean bNow = gamepad1.b;
            if (bNow && !bPressedLast) {
                flywheel.toggleShooterOn();
            }
            bPressedLast = bNow;

            // UPDATE FLYWHEEL
            flywheel.update(nowMs, false);

            // SAVE POINT (A)
            boolean aNow = gamepad1.a;
            if (aNow && !aPressedLast) {
                String point = String.format(
                        "new CalibrationPoint(new Pose(%.0f, %.0f, 0), %.0f),",
                        currentPose.getX(), currentPose.getY(), currentRpm
                );
                collectedPoints.add(point);

                // Feedback
                gamepad1.rumble(300);
            }
            aPressedLast = aNow;

            // PRINT ALL POINTS (Y)
            if (gamepad1.y && !collectedPoints.isEmpty()) {
                telemetry.clearAll();
                telemetry.addLine("\n=== COPY THIS CODE ===\n");
                for (String p : collectedPoints) {
                    telemetry.addLine(p);
                }
                telemetry.addLine("\n======================");
                telemetry.update();
                sleep(3000);  // Show for 3 seconds
            }

            // === DRIVER STATION TELEMETRY ===
            telemetry.addLine("=== CALIBRATION COLLECTOR ===\n");
            telemetry.addData("Position", "(%.1f, %.1f)",
                    currentPose.getX(), currentPose.getY());
            telemetry.addData("Distance to Goal", "%.1f units", distance);
            telemetry.addLine("");
            telemetry.addData("Target RPM", "%.0f", currentRpm);
            telemetry.addData("Actual RPM", "%.0f", flywheel.getCurrentRPM());
            telemetry.addData("Shooter", flywheel.isShooterOn() ? "ON" : "OFF");
            telemetry.addLine("");
            telemetry.addData("Points Saved", collectedPoints.size());
            telemetry.addLine("\nA=Save | Y=Print | B=Shooter");
            telemetry.addLine("DPAD L/R=±50 | DPAD U/D=±10");

            // === PANELS TELEMETRY ===
            panelsTelemetry.debug("X", String.format("%.1f", currentPose.getX()));
            panelsTelemetry.debug("Y", String.format("%.1f", currentPose.getY()));
            panelsTelemetry.debug("Distance", String.format("%.1f", distance));
            panelsTelemetry.debug("Target RPM", String.format("%.0f", currentRpm));
            panelsTelemetry.debug("Actual RPM", String.format("%.0f", flywheel.getCurrentRPM()));
            panelsTelemetry.debug("Shooter", flywheel.isShooterOn() ? "ON" : "OFF");
            panelsTelemetry.debug("Points Saved", collectedPoints.size());

            // Show last saved point in Panels
            if (!collectedPoints.isEmpty()) {
                panelsTelemetry.debug("Last Point", collectedPoints.get(collectedPoints.size() - 1));
            }

            // Update both telemetries
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }

        flywheel.setShooterOn(false);
    }

    private double distanceToGoal(Pose pose) {
        double dx = pose.getX() - GOAL.getX();
        double dy = pose.getY() - GOAL.getY();
        return Math.hypot(dx, dy);
    }
}