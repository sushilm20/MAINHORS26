package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.TurretController;

/**
 * ExperimentalPedroAuto
 *
 * - Waits for shooter to reach close-mode RPM before starting pathing.
 * - Keeps turret auto-align running.
 * - Starts intake/compression before paths 3, 6, 9 (same as teleop).
 * - Ensures intake is RUNNING continuously from before starting path 3 until AFTER reaching path 4,
 *   similarly for the segments 6->7 and 9->10.
 * - Whenever a completed path ENDS at (48,96) the opmode:
 *     1) pauses at that pose for PRE_ACTION_WAIT_SECONDS (1.3s),
 *     2) then starts intake/compression,
 *     3) runs intake/compression for INTAKE_WAIT_SECONDS,
 *     4) runs claw servo action (close then open) automatically,
 *     5) then continues to the next path.
 *
 * - Final path end point is (42.000, 80.000) (heading preserved).
 *
 * Hardware names used (must match your robot): "shooter","turret","intakeMotor",
 * "leftCompressionServo","rightCompressionServo","clawServo","imu"
 */
@Autonomous(name = "Experimental Pedro Pathing Auto", group = "Autonomous")
@Configurable
public class ExperimentalPedroAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // Intake-wait state & timer
    private Timer intakeTimer;
    private static final int INTAKE_WAIT_STATE = 101;
    private static final double INTAKE_WAIT_SECONDS = 3.0; // edit this value to change intake-wait duration

    // Claw action state & timing
    private static final int CLAW_ACTION_STATE = 102;
    private long clawActionStartMs = 0L;
    private int clawActionPhase = 0; // 0 idle, 1 waiting for close duration
    private static final long CLAW_CLOSE_MS = 500L; // same as teleop

    // Pre-action (delay before starting intake/claw) state & timer
    private Timer preActionTimer;
    private static final int PRE_ACTION_STATE = 103;
    private static final double PRE_ACTION_WAIT_SECONDS = 0.8; // changed to 1.3s per request

    // Shooter-wait state before starting paths
    private static final int WAIT_FOR_SHOOTER_STATE = 200;
    private long shooterWaitStartMs = -1;
    private static final long SHOOTER_WAIT_TIMEOUT_MS = 4000L; // fallback timeout if shooter doesn't spin up

    // Shooter / Turret hardware & controllers
    private DcMotor shooterMotor;
    private DcMotor turretMotor;
    private BNO055IMU imu;
    private Flywheel flywheel;
    private TurretController turretController;
    private static final double AUTO_SHOOTER_RPM = 90.0; // close-mode target

    // Intake + compression hardware (from teleop)
    private DcMotor intakeMotor;
    private Servo leftCompressionServo;
    private Servo rightCompressionServo;

    // Claw servo
    private Servo clawServo;

    // Intake/compression "on" values (match teleop right-trigger behavior)
    private static final double INTAKE_ON_POWER = 1.0;
    private static final double LEFT_COMPRESSION_ON = 1.0;
    private static final double RIGHT_COMPRESSION_ON = 0.0;
    private static final double LEFT_COMPRESSION_OFF = 0.5;
    private static final double RIGHT_COMPRESSION_OFF = 0.5;

    // pending next path index used when we enter PRE_ACTION_STATE / INTAKE_WAIT_STATE or CLAW_ACTION_STATE
    private int pendingState = -1;

    // Intake-segment tracking: when starting a multi-path intake segment (3->4, 6->7, 9->10),
    // set intakeSegmentEnd to the path index after which the intake should be stopped (4,7,10).
    // -1 when no active intake segment.
    private int intakeSegmentEnd = -1;

    // helper: which path indices end at the shoot pose (48,96)
    // From your Paths: Path1, Path4, Path7, Path10 end at (48,96)
    private boolean endsAtShoot(int pathIndex) {
        return pathIndex == 1 || pathIndex == 4 || pathIndex == 7 || pathIndex == 10;
    }

    public ExperimentalPedroAuto() {}

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create follower and build paths
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        // Set starting pose to the start point of Path1 (and heading to match interpolation start)
        follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));

        // Timers & state
        intakeTimer = new Timer();
        preActionTimer = new Timer();
        pendingState = -1;
        intakeSegmentEnd = -1;

        // --- Hardware for shooter & turret (match teleop names) ---
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            turretMotor = hardwareMap.get(DcMotor.class, "turret");

            // Directions and modes (same as teleop)
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            turretMotor.setDirection(DcMotor.Direction.FORWARD);

            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Failed to map shooter/turret motors: " + e.getMessage());
        }

        // IMU init (same parameters as teleop)
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
            imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(imuParams);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "IMU not found or failed to init: " + e.getMessage());
        }

        // Instantiate subsystems using the provided classes (pass OpMode telemetry for telemetry output)
        try {
            if (shooterMotor != null) flywheel = new Flywheel(shooterMotor, telemetry);
            if (turretMotor != null) turretController = new TurretController(turretMotor, imu, telemetry);

            // Prepare turret controller just like teleop
            if (turretController != null) {
                turretController.captureReferences();
                turretController.resetPidState();
            }

            // Ensure flywheel default mode (close) and turned on so auto will drive it
            if (flywheel != null) {
                flywheel.setModeFar(false); // set Close mode target (90 rpm)
                flywheel.setShooterOn(true);
                flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Flywheel/TurretController creation error: " + e.getMessage());
        }

        // --- Intake & compression hardware (same names as teleop) ---
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            leftCompressionServo = hardwareMap.get(Servo.class, "leftCompressionServo");
            rightCompressionServo = hardwareMap.get(Servo.class, "rightCompressionServo");

            // Direction per request
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);

            // Set defaults (same as teleop off state)
            intakeMotor.setPower(0.0);
            leftCompressionServo.setPosition(LEFT_COMPRESSION_OFF);
            rightCompressionServo.setPosition(RIGHT_COMPRESSION_OFF);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake/compression mapping failed: " + e.getMessage());
        }

        // --- Claw servo ---
        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            // ensure default open position as teleop uses 0.63 for open
            clawServo.setPosition(0.63);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Claw servo mapping failed: " + e.getMessage());
        }

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Warm up flywheel readings while waiting for start
        if (flywheel != null) {
            flywheel.update(System.currentTimeMillis(), false);
        }
    }

    @Override
    public void start() {
        // Start spinner and turret references
        if (flywheel != null) {
            flywheel.setShooterOn(true);
            flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
        }
        if (turretController != null) {
            turretController.captureReferences();
            turretController.resetPidState();
        }

        // Start measuring shooter-wait
        shooterWaitStartMs = System.currentTimeMillis();

        // Enter WAIT_FOR_SHOOTER_STATE so we ensure shooter reaches RPM before moving
        setPathState(WAIT_FOR_SHOOTER_STATE);
    }

    @Override
    public void loop() {
        // Keep Pedro Pathing updated
        follower.update();

        long nowMs = System.currentTimeMillis();

        // Update flywheel (closed-loop) each loop so it runs for the whole OpMode
        if (flywheel != null) {
            flywheel.handleLeftTrigger(false);
            flywheel.update(nowMs, false);
        }

        // Run turret auto-alignment exactly as teleop: manual flag false in autonomous
        if (turretController != null) {
            turretController.update(false, 0.0);
        }

        // Run the FSM
        pathState = autonomousPathUpdate();

        // Panels & driver station telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        if (flywheel != null) {
            panelsTelemetry.debug("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
            panelsTelemetry.debug("Fly Target", String.format("%.1f", flywheel.getTargetRPM()));
            panelsTelemetry.debug("Fly On", flywheel.isShooterOn());
            panelsTelemetry.debug("Fly AtTarget", flywheel.isAtTarget());
        }
        if (turretMotor != null && turretController != null) {
            panelsTelemetry.debug("Turret Enc", turretMotor.getCurrentPosition());
            panelsTelemetry.debug("Turret Power", turretController.getLastAppliedPower());
        }
        if (intakeMotor != null) {
            panelsTelemetry.debug("Intake Power", intakeMotor.getPower());
            panelsTelemetry.debug("LeftCompPos", leftCompressionServo != null ? leftCompressionServo.getPosition() : -1);
            panelsTelemetry.debug("RightCompPos", rightCompressionServo != null ? rightCompressionServo.getPosition() : -1);
        }
        if (clawServo != null) {
            panelsTelemetry.debug("ClawPos", clawServo.getPosition());
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        // Stop shooter and turret safely
        if (flywheel != null) {
            flywheel.setShooterOn(false);
            flywheel.update(System.currentTimeMillis(), false);
        }
        if (turretController != null) {
            turretController.update(true, 0.0); // manual with 0 power to ensure motor stops
        }

        // Ensure intake off and claw open
        stopIntake();
        if (clawServo != null) clawServo.setPosition(0.63);
    }

    // --- Intake helpers (right-trigger behavior) ---
    private void startIntake() {
        try {
            if (intakeMotor != null) intakeMotor.setPower(INTAKE_ON_POWER);
            if (leftCompressionServo != null) leftCompressionServo.setPosition(LEFT_COMPRESSION_ON);
            if (rightCompressionServo != null) rightCompressionServo.setPosition(RIGHT_COMPRESSION_ON);
        } catch (Exception e) {
            panelsTelemetry.debug("Intake", "startIntake error: " + e.getMessage());
        }
    }

    private void stopIntake() {
        try {
            if (intakeMotor != null) intakeMotor.setPower(0.0);
            if (leftCompressionServo != null) leftCompressionServo.setPosition(LEFT_COMPRESSION_OFF);
            if (rightCompressionServo != null) rightCompressionServo.setPosition(RIGHT_COMPRESSION_OFF);
        } catch (Exception e) {
            panelsTelemetry.debug("Intake", "stopIntake error: " + e.getMessage());
        }
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
        Path1 = follower
          .pathBuilder()
          .addPath(
            new BezierLine(new Pose(20.000, 122.000), new Pose(48.000, 96.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(130))
          .build();

        Path2 = follower
          .pathBuilder()
          .addPath(
            new BezierLine(new Pose(48.000, 96.000), new Pose(42.000, 84.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
          .build();
            
        Path3 = follower
          .pathBuilder()
          .addPath(
            new BezierLine(new Pose(42.000, 84.000), new Pose(26.000, 84.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
          .build();

        Path4 = follower
            .pathBuilder()
            .addPath(
            new BezierLine(new Pose(26.000, 84.000), new Pose(48.000, 96.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
            .build();

        Path5 = follower
          .pathBuilder()
          .addPath(
            new BezierLine(new Pose(48.000, 96.000), new Pose(46.000, 57.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
          .build();

        Path6 = follower
          .pathBuilder()
          .addPath(
            new BezierLine(new Pose(46.000, 57.000), new Pose(16.000, 57.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
          .build();

        Path7 = follower
          .pathBuilder()
          .addPath(
            new BezierLine(new Pose(16.000, 57.000), new Pose(48.000, 96.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
          .build();

        Path8 = follower
          .pathBuilder()
          .addPath(
              new BezierLine(new Pose(48.000, 96.000), new Pose(43.000, 36.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
          .build();

        Path9 = follower
          .pathBuilder()
          .addPath(
            new BezierLine(new Pose(43.000, 36.000), new Pose(16.000, 36.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
          .build();

        Path10 = follower
          .pathBuilder()
          .addPath(
            new BezierLine(new Pose(16.000, 36.000), new Pose(48.000, 96.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
          .build();

        Path11 = follower
          .pathBuilder()
          .addPath(
            new BezierLine(new Pose(48.000, 96.000), new Pose(20.000, 122.000))
          )
          .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(135))
          .build();
      }
    }

    public int autonomousPathUpdate() {
        /*
          FSM that:
           - waits for shooter to reach close-mode RPM (or timeout) before starting paths,
           - follows Path1 -> Path2 -> ... -> Path11 sequentially,
           - whenever the robot finishes a path that ENDS at (48,96) we:
               a) wait PRE_ACTION_WAIT_SECONDS (1.3s),
               b) start intake for INTAKE_WAIT_SECONDS,
               c) run claw action (close then open),
               d) continue,
           - intake is also explicitly turned ON BEFORE starting path segments (3->4, 6->7, 9->10).
        */
        switch (pathState) {
            case WAIT_FOR_SHOOTER_STATE:
                // Wait until flywheel reports at-target OR timeout
                boolean atTarget = (flywheel != null && flywheel.isAtTarget());
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (System.currentTimeMillis() - shooterWaitStartMs);
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) {
                    // proceed to first path immediately
                    followPathIndexAndSetState(1);
                }
                break;

            case PRE_ACTION_STATE:
                // wait PRE_ACTION_WAIT_SECONDS, then start intake-wait sequence
                if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                    // start intake for INTAKE_WAIT_SECONDS, then claw action will run
                    startIntake();
                    intakeTimer.resetTimer();
                    setPathState(INTAKE_WAIT_STATE);
                }
                break;

            case INTAKE_WAIT_STATE:
                // wait for INTAKE_WAIT_SECONDS, then stop intake and proceed to CLAW action
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_SECONDS) {
                    stopIntake();
                    // begin claw action (close then open) automatically
                    if (clawServo != null) clawServo.setPosition(0.2); // close
                    clawActionPhase = 1;
                    clawActionStartMs = System.currentTimeMillis();
                    setPathState(CLAW_ACTION_STATE);
                }
                break;

            case CLAW_ACTION_STATE:
                // wait for CLAW_CLOSE_MS then open claw and continue to pendingState
                if (clawActionPhase == 1) {
                    long nowMs = System.currentTimeMillis();
                    if (nowMs >= clawActionStartMs + CLAW_CLOSE_MS) {
                        if (clawServo != null) clawServo.setPosition(0.63); // open
                        clawActionPhase = 0;
                        // continue to next path
                        if (pendingState > 0) {
                            followPathIndexAndSetState(pendingState);
                        } else {
                            setPathState(-1);
                        }
                    }
                }
                break;

            // Each path state waits for follower to finish. When finished:
            // - if the completed path ENDS at shoot pose, enter PRE_ACTION_STATE (1.3s) then INTAKE_WAIT_STATE then CLAW_ACTION_STATE;
            // - otherwise continue to the next path immediately (also stop intake if the intake segment ended).
            case 1:
                if (!follower.isBusy()) {
                    int next = 2;
                    if (endsAtShoot(1)) {
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                    } else {
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    int next = 3;
                    if (endsAtShoot(2)) {
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                    } else {
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    // path3-specific: we started intake before path3 and we want it to continue through path4.
                    int next = 4;
                    // If the path ends at shoot pose, we will run the PRE_ACTION_STATE instead; otherwise, continue to next.
                    if (endsAtShoot(3)) {
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                    } else {
                        // DO NOT stop intake here (intentional: keep intake running until path4 completes).
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    int next = 5;
                    if (endsAtShoot(4)) {
                        // robot ended at shoot pose: wait PRE_ACTION_WAIT_SECONDS then intake-wait, etc.
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                        // clear intake-segment tracking now that we've reached its end
                        intakeSegmentEnd = -1;
                    } else {
                        // if this is the end of an intake segment, stop intake now
                        if (intakeSegmentEnd == 4) {
                            stopIntake();
                            intakeSegmentEnd = -1;
                        }
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    int next = 6;
                    if (endsAtShoot(5)) {
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                    } else {
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    int next = 7;
                    if (endsAtShoot(6)) {
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                    } else {
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    int next = 8;
                    if (endsAtShoot(7)) {
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                        intakeSegmentEnd = -1;
                    } else {
                        if (intakeSegmentEnd == 7) {
                            stopIntake();
                            intakeSegmentEnd = -1;
                        }
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    int next = 9;
                    if (endsAtShoot(8)) {
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                    } else {
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    int next = 10;
                    if (endsAtShoot(9)) {
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                    } else {
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    int next = 11;
                    if (endsAtShoot(10)) {
                        pendingState = next;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                        intakeSegmentEnd = -1;
                    } else {
                        if (intakeSegmentEnd == 10) {
                            stopIntake();
                            intakeSegmentEnd = -1;
                        }
                        followPathIndexAndSetState(next);
                    }
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    // last path done
                    if (endsAtShoot(11)) {
                        pendingState = -1;
                        preActionTimer.resetTimer();
                        setPathState(PRE_ACTION_STATE);
                    } else {
                        setPathState(-1);
                    }
                }
                break;

            default:
                // idle/finished (-1 or other)
                break;
        }

        return pathState;
    }

    /**
     * Helper to follow a path by its index and set pathState accordingly.
     * Also turns intake ON immediately before starting specific path segments (3->4, 6->7, 9->10).
     * Sets intakeSegmentEnd so intake won't be stopped until the end path completes.
     */
    private void followPathIndexAndSetState(int idx) {
        switch (idx) {
            case 1:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 2:
                follower.followPath(paths.Path2);
                setPathState(2);
                break;
            case 3:
                // turn on intake/compression BEFORE starting path 3 and keep it running through path4
                intakeSegmentEnd = 4;
                startIntake();
                follower.followPath(paths.Path3);
                setPathState(3);
                break;
            case 4:
                follower.followPath(paths.Path4);
                setPathState(4);
                break;
            case 5:
                follower.followPath(paths.Path5);
                setPathState(5);
                break;
            case 6:
                // turn on intake/compression BEFORE starting path 6 and keep it running through path7
                intakeSegmentEnd = 7;
                startIntake();
                follower.followPath(paths.Path6);
                setPathState(6);
                break;
            case 7:
                follower.followPath(paths.Path7);
                setPathState(7);
                break;
            case 8:
                follower.followPath(paths.Path8);
                setPathState(8);
                break;
            case 9:
                // turn on intake/compression BEFORE starting path 9 and keep it running through path10
                intakeSegmentEnd = 10;
                startIntake();
                follower.followPath(paths.Path9);
                setPathState(9);
                break;
            case 10:
                follower.followPath(paths.Path10);
                setPathState(10);
                break;
            case 11:
                follower.followPath(paths.Path11);
                setPathState(11);
                break;
            default:
                setPathState(-1);
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
    }
}
