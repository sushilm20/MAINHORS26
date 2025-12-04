package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class pedroauto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = -1; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Timers used by the FSM (mirrors ExampleAuto structure)
    private Timer pathTimer;
    private Timer opmodeTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // no-op; kept for parity with ExampleAuto
    }

    @Override
    public void start() {
        // Called once when play is pressed. Reset timers and start FSM at state 0.
        if (opmodeTimer != null) opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        // Terminal cleanup if necessary
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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17.742, 128.527), new Pose(60.447, 85.203))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(142.5),
                            Math.toRadians(180)
                    )
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.447, 85.203), new Pose(18.468, 84.104))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.468, 84.104), new Pose(44.672, 99.078))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(142.5)
                    )
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.672, 99.078), new Pose(44.423, 59.896))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(142.5),
                            Math.toRadians(180)
                    )
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.423, 59.896), new Pose(18.468, 59.646))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.468, 59.646), new Pose(44.922, 99.078))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(142.5)
                    )
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.922, 99.078), new Pose(43.924, 35.438))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(142.5),
                            Math.toRadians(180)
                    )
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.924, 35.438), new Pose(18.718, 35.438))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.718, 35.438), new Pose(44.922, 99.328))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(142.5)
                    )
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.922, 99.328), new Pose(44.672, 134.017))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(142.5),
                            Math.toRadians(90)
                    )
                    .build();
        }
    }

    /**
     * Finite State Machine following the ExampleAuto logic.
     * - Starts Path1 in state 0.
     * - Waits for follower.isBusy() to be false before advancing to the next path.
     * - Uses follower.followPath(path, true) to hold at the end of a PathChain when appropriate (so you can run actions).
     * - Final state is -1 (terminal).
     */
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start the first path (no hold)
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                // Wait until path1 finishes, then go to Path2 and hold (so actions could run)
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    // Sequence complete; terminal state
                    setPathState(-1);
                }
                break;

            case -1:
            default:
                // Terminal/default: nothing to do
                break;
        }

        return pathState;
    }

    /** Change path state and reset the path timer (mirrors ExampleAuto behavior). **/
    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) {
            pathTimer.resetTimer();
        }
        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Set Path State", pathState);
        }
    }
}