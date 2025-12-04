package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTagTurretTracker OpMode
 * 
 * This OpMode uses webcam1 to detect AprilTag 22 and automatically 
 * centers the turret on the tag. The turret motor has limits set 
 * to -400 and 400 encoder positions.
 * 
 * Controls:
 * - The turret will automatically track and center on AprilTag 22
 * - Press DPAD_DOWN to manually stop the turret
 */
@TeleOp(name="AprilTag Turret Tracker", group="TeleOp")
public class AprilTagTurretTracker extends LinearOpMode {
    
    private DcMotor turretMotor;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private TurretTracker turretTracker;
    
    // Target AprilTag ID
    private static final int TARGET_TAG_ID = 22;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        
        // Initialize AprilTag detection
        initializeAprilTag();
        
        // Create turret tracker
        turretTracker = new TurretTracker(turretMotor, TARGET_TAG_ID);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target AprilTag", TARGET_TAG_ID);
        telemetry.addData("Turret Limits", "-400 to 400");
        telemetry.addData(">", "Press START to begin");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get the latest AprilTag detections
                AprilTagDetection targetDetection = findTargetTag();
                
                // Manual stop control
                if (gamepad1.dpad_down) {
                    turretTracker.stop();
                    telemetry.addData("Mode", "Manual Stop");
                } else {
                    // Update turret based on AprilTag detection
                    String status = turretTracker.updateTurret(targetDetection);
                    telemetry.addData("Status", status);
                }
                
                // Display information
                displayTelemetry(targetDetection);
                
                // Update telemetry
                telemetry.update();
                
                // Save CPU resources
                sleep(20);
            }
        }
        
        // Clean up
        visionPortal.close();
    }
    
    /**
     * Initialize hardware components
     */
    private void initializeHardware() {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    
    /**
     * Initialize AprilTag processor and vision portal
     */
    private void initializeAprilTag() {
        // Create the AprilTag processor
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        
        // Create the vision portal using webcam1
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "webcam1"), aprilTag);
    }
    //test
    /**
     * Find the target AprilTag in the current detections
     * @return The target AprilTag detection, or null if not found
     */
    private AprilTagDetection findTargetTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == TARGET_TAG_ID) {
                return detection;
            }
        }
        
        return null;
    }
    
    /**
     * Display telemetry information
     * @param detection The current target tag detection (can be null)
     */
    private void displayTelemetry(AprilTagDetection detection) {
        telemetry.addData("# AprilTags Detected", aprilTag.getDetections().size());
        telemetry.addData("Turret Position", turretTracker.getCurrentPosition());
        telemetry.addData("Within Limits", turretTracker.isWithinLimits());
        
        if (detection != null) {
            telemetry.addLine("\n=== Target Tag Found ===");
            telemetry.addData("Tag ID", detection.id);
            telemetry.addData("Tag X Position", "%.1f", detection.center.x);
            telemetry.addData("Tag Y Position", "%.1f", detection.center.y);
            
            if (detection.metadata != null) {
                telemetry.addData("Tag Name", detection.metadata.name);
            }
        } else {
            telemetry.addLine("\n=== Target Tag Not Found ===");
            telemetry.addData("Looking for Tag", TARGET_TAG_ID);
        }
        
        telemetry.addLine("\nControls:");
        telemetry.addData("DPAD_DOWN", "Manual Stop");
    }
}
