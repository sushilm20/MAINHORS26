package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class pedroauto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // State machine helpers
    private boolean pathFollowing;
    private long pathStartTimeMs;
    private static final long PATH_TIMEOUT_MS = 8000; // fallback timeout per path

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // keep same starting pose as in the example
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        pathState = 0;
        pathFollowing = false;
        pathStartTimeMs = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine (updates internal state)

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Path Following", pathFollowing);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            // Points replaced with user-provided sequence while keeping original heading interpolations

            // 1) 25,83 -> 39,98
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(25.000, 72.000), new Pose(39.000, 98.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(130))
                    .build();

            // 2) 39,98 -> 35,72 (collect -> shoot)
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(39.000, 98.000), new Pose(35.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            // 3) 35,72 -> 8,101 (shoot -> collect 2nd)
            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(35.000, 72.000), new Pose(8.000, 101.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // 4) 8,101 -> 23,115 (collect 2nd -> collect 3 2nd time)
            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8.000, 101.000), new Pose(23.000, 115.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            // 5) 23,115 -> 35,72 (collect -> shoot)
            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.000, 115.000), new Pose(35.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            // 6) 35,72 -> 9,115 (shoot -> collect 3rd)
            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(35.000, 72.000), new Pose(9.000, 115.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // 7) 9,115 -> 4,131 (collect 3rd -> collected 3)
            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(9.000, 115.000), new Pose(4.000, 131.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            // 8) 4,131 -> 35,72 (back to shoot)
            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(4.000, 131.000), new Pose(35.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            // 9) 35,72 -> 29,88
            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(35.000, 72.000), new Pose(29.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // 10) 29,88 -> 35,72 (return to shoot or reposition)
            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(29.000, 88.000), new Pose(35.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            // 11) Final: stay at shoot pose but rotate to final heading (uses original Path11 heading interpolation)
            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(35.000, 72.000), new Pose(35.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(270))
                    .build();
        }
    }

    /**
     * Simple sequential state machine to run the PathChains in the order defined above.
     *
     * Behavior:
     * - When entering a state it starts following the corresponding PathChain.
     * - It waits until the follower reports it is no longer following (path finished),
     *   or a timeout occurs, then advances to the next state.
     *
     * Note: This code assumes the Follower API exposes:
     *   follower.follow(PathChain) to begin following a path
     *   follower.isFollowing() to query whether a path is actively being followed
     *
     * If your Follower implementation uses different method names, adjust the calls accordingly.
     */
    public int autonomousPathUpdate() {
        long now = System.currentTimeMillis();

        // Helper lambda-style behavior using methods (kept inline for clarity)
        switch (pathState) {
            case 0:
                if (!pathFollowing) {
                    follower.followPath(paths.Path1);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    // if follower finished or timed out -> advance
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 1:
                if (!pathFollowing) {
                    follower.followPath(paths.Path2);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 2:
                if (!pathFollowing) {
                    follower.followPath(paths.Path3);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 3:
                if (!pathFollowing) {
                    follower.followPath(paths.Path4);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 4:
                if (!pathFollowing) {
                    follower.followPath(paths.Path5);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 5:
                if (!pathFollowing) {
                    follower.followPath(paths.Path6);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 6:
                if (!pathFollowing) {
                    follower.followPath(paths.Path7);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 7:
                if (!pathFollowing) {
                    follower.followPath(paths.Path8);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 8:
                if (!pathFollowing) {
                    follower.followPath(paths.Path9);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 9:
                if (!pathFollowing) {
                    follower.followPath(paths.Path10);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            case 10:
                if (!pathFollowing) {
                    follower.followPath(paths.Path11);
                    pathFollowing = true;
                    pathStartTimeMs = now;
                } else {
                    if (!follower.getFollowingPathChain() || now - pathStartTimeMs > PATH_TIMEOUT_MS) {
                        // Finished final path -> move to terminal state
                        pathFollowing = false;
                        pathState++;
                    }
                }
                break;

            default:
                // Terminal state: do nothing (all paths completed)
                break;
        }

        return pathState;
    }
}