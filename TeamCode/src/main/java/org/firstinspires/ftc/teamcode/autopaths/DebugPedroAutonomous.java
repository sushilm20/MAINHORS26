package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Debug OpMode:
 * 1) Builds a short forward path (small delta x)
 * 2) Follows it and waits until follower reports not busy
 * 3) Pauses, then follows the reversed path (start/end swapped)
 * Use telemetry to determine whether the robot actually moved forward or backward
 * and whether pose readings changed in the expected sign/direction.
 */
@Autonomous(name = "Pedro Debug: Forward vs Reverse Test", group = "Debug")
public class DebugPedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int state = 0;
    private PathChain forwardPath;
    private PathChain reversedPath;
    private long stateStartTime = 0L;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create follower and set starting pose to match your match-field pose
        follower = Constants.createFollower(hardwareMap);
        // If your robot physically faces "down" the field, you may need a -90 instead of +90
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        // Build a very small test path (4 inches forward along X)
        forwardPath = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(72, 8), new Pose(68, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        // Reversed path: start and end swapped
        reversedPath = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(68, 8), new Pose(72, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        panelsTelemetry.debug("Status", "Debug initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        // Always update the follower every loop
        follower.update();

        // Basic state machine:
        // 0 -> start forwardPath
        // 1 -> wait for forwardPath to finish, then log
        // 2 -> pause 1s
        // 3 -> start reversedPath
        // 4 -> wait for reversedPath to finish, then done
        switch (state) {
            case 0:
                stateStartTime = System.currentTimeMillis();
                if (!follower.isBusy()) {
                    follower.followPath(forwardPath);
                }
                state = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    // Log after forward
                    panelsTelemetry.debug("After forward - X", follower.getPose().getX());
                    panelsTelemetry.debug("After forward - Y", follower.getPose().getY());
                    panelsTelemetry.debug("After forward - Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
                    panelsTelemetry.update(telemetry);

                    stateStartTime = System.currentTimeMillis();
                    state = 2;
                }
                break;

            case 2:
                // short pause so we can observe
                if (System.currentTimeMillis() - stateStartTime > 1000) {
                    state = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(reversedPath);
                }
                state = 4;
                break;

            case 4:
                if (!follower.isBusy()) {
                    // Log after reverse attempt
                    panelsTelemetry.debug("After reverse - X", follower.getPose().getX());
                    panelsTelemetry.debug("After reverse - Y", follower.getPose().getY());
                    panelsTelemetry.debug("After reverse - Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
                    panelsTelemetry.update(telemetry);

                    // All done, remain in final state
                    state = 99;
                }
                break;

            default:
                // Idle
                break;
        }

        // continuous telemetry so you can watch live pose and state
        panelsTelemetry.debug("State", state);
        panelsTelemetry.debug("Live X", follower.getPose().getX());
        panelsTelemetry.debug("Live Y", follower.getPose().getY());
        panelsTelemetry.debug("Live Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }
}