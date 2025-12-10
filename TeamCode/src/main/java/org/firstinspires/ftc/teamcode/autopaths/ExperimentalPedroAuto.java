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
 * ExperimentalPedroAuto (improved PRE_ACTION pose handling + fallback)
 *
 * Fixes:
 *  - PRE_ACTION timer only begins after the robot reaches the shoot pose OR after a short pose-wait timeout.
 *  - Increased default pose tolerance and added debug telemetry for distance-to-target so you can tune.
 *  - Prevents the robot from stalling indefinitely when it gets "very close but not exact".
 *
 * Behavior preserved otherwise.
 *
 * Modifications:
 *  - When path 4, path 7 or path 10 starts, run intake for a timed 1.0s and then stop.
 *  - PRE_ACTION_WAIT_SECONDS reduced by 0.5s (from 0.8 to 0.3).
 *  - Turret remains present and tracked but is forced into manual 0 power mode each loop (no movement).
 *  - Shooter does NOT spin up during init(); it will be started in start().
 */
@Autonomous(name = "Experimental Pedro Pathing Auto", group = "Autonomous")
@Configurable
public class ExperimentalPedroAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // State machine
    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, PRE_ACTION, INTAKE_WAIT, CLAW_ACTION, FINISHED }
    private AutoState state = AutoState.IDLE;

    // current path index being run (1..11). 0 when none.
    private int currentPathIndex = 0;
    // next path index to run after PRE_ACTION/CLAW sequences
    private int nextPathIndex = -1;

    // Intake-wait state & timer
    private Timer intakeTimer;
    private static final double INTAKE_WAIT_SECONDS = 2.5; // change to shorten/lengthen intake duration

    // Timed intake on-path start (for path4, path7 and path10)
    private Timer timedIntakeTimer;
    private static final double TIMED_INTAKE_SECONDS = 1.0; // run intake for 1 second when path4, path7 or path10 starts
    private boolean timedIntakeActive = false;

    // Claw action state & timing
    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 250L; // duration to hold claw closed

    // Pre-action (delay before starting intake/claw) timer
    private Timer preActionTimer;
    private static final double PRE_ACTION_WAIT_SECONDS = 0.3; // lowered by 0.5s from 0.8

    // Pose-wait timer (wait for robot to reach pose before starting PRE_ACTION). Fallback if it never quite reaches it.
    private Timer poseWaitTimer;
    private static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3; // fallback after short timeout

    // Flag to indicate whether PRE_ACTION timer has been started (we only start it when robot reaches pose or fallback triggers)
    private boolean preActionTimerStarted = false;
    // Flag set when entering PRE_ACTION state to reset poseWaitTimer
    private boolean preActionEntered = false;

    // Shooter-wait state before starting paths
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

    // Intake-segment tracking: when starting a multi-path intake segment (3, 6, 9),
    // set intakeSegmentEnd to the path index after which the intake should be stopped.
    // -1 when no active intake segment.
    private int intakeSegmentEnd = -1;

    // Shoot pose constants and tolerance - used to ensure PRE_ACTION timer starts only when robot reaches pose
    private static final double SHOOT_POSE_X = 48.0;
    private static final double SHOOT_POSE_Y = 96.0;
    private static final double START_POSE_TOLERANCE_IN = 6.0; // increased tolerance to avoid tiny-miss stalls

    // Turret movement control flag (false = allow automatic movement; true = manual mode with 0 power)
    // We will force manual=true in loop so turret does not move but controller still updates/reads sensors.
    private final boolean turretForceManualNoMove = true;

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
        timedIntakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        nextPathIndex = -1;
        intakeSegmentEnd = -1;
        preActionTimerStarted = false;
        preActionEntered = false;
        timedIntakeActive = false;

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
            // ensure brake mode so turret holds position when power = 0
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

            // NOTE: Do NOT turn the shooter on during init().
            // Keep flywheel subsystem created so readings are available, but don't enable spinning
            // until the autonomous actually starts (start()).
            if (flywheel != null) {
                flywheel.setModeFar(false); // set Close mode target mode if needed, but do not enable shooter
                // intentionally do NOT call flywheel.setShooterOn(true) or setTargetRPM here
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

        panelsTelemetry.debug("Status", "Initialized (shooter remains OFF until start())");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Warm up flywheel readings while waiting for start
        if (flywheel != null) {
            flywheel.update(System.currentTimeMillis(), false);
        }
        // Keep turret controller state refreshed during init loop as well
        if (turretController != null) {
            // Keep in manual 0-power mode during init loop so it doesn't move
            turretController.update(true, 0.0);
        }
    }

    @Override
    public void start() {
        // Start spinner and turret references at the beginning of autonomous
        if (flywheel != null) {
            flywheel.setShooterOn(true);            // now enable shooter
            flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
        }
        if (turretController != null) {
            turretController.captureReferences();
            turretController.resetPidState();
        }

        // Start measuring shooter-wait
        shooterWaitStartMs = System.currentTimeMillis();
        state = AutoState.WAIT_FOR_SHOOTER;
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

        // Turret: we keep the controller active so it continues to read sensors/encoders and update internal state,
        // but we force manual mode with 0 power so it won't apply motion. This leaves the motor in BRAKE and
        // preserves heading/position telemetry.
        if (turretController != null) {
            turretController.update(true, 0.0);
        }

        // Run the refined FSM
        runStateMachine(nowMs);

        // Panels & driver station telemetry (including distance-to-target for tuning)
        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
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
            // show that turret is forced manual/no-move
            panelsTelemetry.debug("TurretManualNoMove", String.valueOf(turretForceManualNoMove));
        }
        if (intakeMotor != null) {
            panelsTelemetry.debug("Intake Power", intakeMotor.getPower());
            panelsTelemetry.debug("LeftCompPos", leftCompressionServo != null ? leftCompressionServo.getPosition() : -1);
            panelsTelemetry.debug("RightCompPos", rightCompressionServo != null ? rightCompressionServo.getPosition() : -1);
        }
        if (clawServo != null) {
            panelsTelemetry.debug("ClawPos", clawServo.getPosition());
        }

        double dist = distanceToShootPose();
        panelsTelemetry.debug("DistToShootPose", String.format("%.2f", dist));
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
            // manual with 0 power to ensure motor stops and stays braked
            turretController.update(true, 0.0);
        }

        // Ensure intake off and claw open
        stopIntake();
        if (clawServo != null) clawServo.setPosition(0.63);
        state = AutoState.FINISHED;
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

    // Helper to check which path indices end at the shoot pose (48,96)
    private boolean endsAtShoot(int pathIndex) {
        return pathIndex == 1 || pathIndex == 4 || pathIndex == 7 || pathIndex == 10;
    }

    /**
     * Returns true when follower's current pose is within tolerance (inches) of target.
     */
    private boolean isAtPose(double targetX, double targetY, double tolerance) {
        try {
            Pose p = follower.getPose();
            double dx = p.getX() - targetX;
            double dy = p.getY() - targetY;
            return Math.hypot(dx, dy) <= tolerance;
        } catch (Exception e) {
            panelsTelemetry.debug("isAtPose", "error: " + e.getMessage());
            return false;
        }
    }

    /**
     * Compute Euclidean distance to shoot pose for telemetry/tuning.
     */
    private double distanceToShootPose() {
        try {
            Pose p = follower.getPose();
            double dx = p.getX() - SHOOT_POSE_X;
            double dy = p.getY() - SHOOT_POSE_Y;
            return Math.hypot(dx, dy);
        } catch (Exception e) {
            return Double.POSITIVE_INFINITY;
        }
    }

    /**
     * Start a path by index and set the state to RUNNING_PATH.
     * This helper also starts intake/compression before specific path segments (3, 6, 9).
     * Intake will run only DURING those starting paths and will be stopped when that path finishes,
     * so it will not be active during paths 4, 7, 10 by default.
     *
     * Additionally: when starting path 4, path 7 or path 10 we start a timed intake that runs for 1s and then stops.
     */
    private void startPath(int idx) {
        if (idx < 1 || idx > 11) {
            currentPathIndex = 0;
            state = AutoState.FINISHED;
            return;
        }

        // If this path is the beginning of an intake segment, enable intake and record its end index.
        // We set intakeSegmentEnd to the same idx so intake runs only during that path and is stopped
        // when that path finishes.
        if (idx == 3) {
            intakeSegmentEnd = 3;
            startIntake();
        } else if (idx == 6) {
            intakeSegmentEnd = 6;
            startIntake();
        } else if (idx == 9) {
            intakeSegmentEnd = 9;
            startIntake();
        }

        // Special: start a timed intake (1s) when path 4, 7 or 10 starts.
        if (idx == 4 || idx == 7 || idx == 10) {
            // If an intake is already running due to another reason, this will simply ensure it stays on.
            startIntake();
            timedIntakeTimer.resetTimer();
            timedIntakeActive = true;
            panelsTelemetry.debug("TimedIntake", "Started timed intake for path " + idx);
        }

        // Follow the requested path
        switch (idx) {
            case 1: follower.followPath(paths.Path1); break;
            case 2: follower.followPath(paths.Path2); break;
            case 3: follower.followPath(paths.Path3); break;
            case 4: follower.followPath(paths.Path4); break;
            case 5: follower.followPath(paths.Path5); break;
            case 6: follower.followPath(paths.Path6); break;
            case 7: follower.followPath(paths.Path7); break;
            case 8: follower.followPath(paths.Path8); break;
            case 9: follower.followPath(paths.Path9); break;
            case 10: follower.followPath(paths.Path10); break;
            case 11: follower.followPath(paths.Path11); break;
            default: break;
        }

        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
    }

    private void runStateMachine(long nowMs) {
        // Handle timed intake expiration independent of the FSM states:
        if (timedIntakeActive) {
            if (timedIntakeTimer.getElapsedTimeSeconds() >= TIMED_INTAKE_SECONDS) {
                // stop the timed intake
                stopIntake();
                timedIntakeActive = false;
                // ensure intakeSegment tracking is cleared so we don't try to stop it again later
                intakeSegmentEnd = -1;
                panelsTelemetry.debug("TimedIntake", "Timed intake ended after " + TIMED_INTAKE_SECONDS + "s");
            } else {
                // still within timed intake period - show telemetry
                panelsTelemetry.debug("TimedIntake", String.format("remaining=%.2fs", TIMED_INTAKE_SECONDS - timedIntakeTimer.getElapsedTimeSeconds()));
            }
        }

        switch (state) {
            case WAIT_FOR_SHOOTER:
                boolean atTarget = (flywheel != null && flywheel.isAtTarget());
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (System.currentTimeMillis() - shooterWaitStartMs);
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) {
                    // proceed to first path immediately
                    startPath(1);
                }
                break;

            case RUNNING_PATH:
                // Wait until follower finishes this path
                if (!follower.isBusy()) {
                    int finished = currentPathIndex;
                    // If finished path was the end of an intake segment, clear tracking and stop intake now
                    if (intakeSegmentEnd == finished) {
                        stopIntake();
                        intakeSegmentEnd = -1;
                    }

                    // If this path ends at the shoot pose, go to PRE_ACTION but do not start the PRE_ACTION timer yet.
                    if (endsAtShoot(finished)) {
                        nextPathIndex = finished + 1;
                        preActionTimerStarted = false; // ensure timer is not considered started
                        preActionEntered = false; // ensure pose wait is reset on entry
                        state = AutoState.PRE_ACTION;
                    } else {
                        // not a shoot-end, just continue to next path (or finish)
                        int next = finished + 1;
                        if (next > 11) {
                            state = AutoState.FINISHED;
                        } else {
                            startPath(next);
                        }
                    }
                }
                break;

            case PRE_ACTION:
                // On first tick after entering PRE_ACTION, reset the poseWaitTimer
                if (!preActionEntered) {
                    poseWaitTimer.resetTimer();
                    preActionTimerStarted = false;
                    preActionEntered = true;
                    panelsTelemetry.debug("PRE_ACTION", "Entered PRE_ACTION, starting pose-wait");
                }

                // If PRE_ACTION timer hasn't been started yet, wait until robot reaches pose OR fallback after a short timeout
                if (!preActionTimerStarted) {
                    double dist = distanceToShootPose();
                    if (dist <= START_POSE_TOLERANCE_IN) {
                        preActionTimer.resetTimer();
                        preActionTimerStarted = true;
                        panelsTelemetry.debug("PRE_ACTION", "At pose: starting PRE_ACTION timer");
                    } else if (poseWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_MAX_POSE_WAIT_SECONDS) {
                        // fallback: start PRE_ACTION timer even if we're not perfectly within tolerance
                        preActionTimer.resetTimer();
                        preActionTimerStarted = true;
                        panelsTelemetry.debug("PRE_ACTION", "Pose-wait timeout: starting PRE_ACTION timer anyway (dist=" + String.format("%.2f", dist) + ")");
                    } else {
                        // Still waiting for pose; remain in PRE_ACTION
                        panelsTelemetry.debug("PRE_ACTION", "Waiting for pose (dist=" + String.format("%.2f", dist) + ")");
                    }
                } else {
                    // PRE_ACTION timer started; wait required PRE_ACTION_WAIT_SECONDS from this moment
                    if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                        // start intake for INTAKE_WAIT_SECONDS, then claw action will run
                        startIntake();
                        intakeTimer.resetTimer();
                        state = AutoState.INTAKE_WAIT;
                    }
                }
                break;

            case INTAKE_WAIT:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_SECONDS) {
                    // Stop intake only if it's not part of a longer intake-segment
                    if (intakeSegmentEnd == -1) {
                        stopIntake();
                    }
                    // begin claw action (close)
                    if (clawServo != null) clawServo.setPosition(0.2); // close
                    clawActionStartMs = System.currentTimeMillis();
                    state = AutoState.CLAW_ACTION;
                }
                break;

            case CLAW_ACTION:
                if (System.currentTimeMillis() >= clawActionStartMs + CLAW_CLOSE_MS) {
                    // open claw and continue to nextPathIndex
                    if (clawServo != null) clawServo.setPosition(0.63); // open
                    if (nextPathIndex > 0 && nextPathIndex <= 11) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;

            case FINISHED:
                // idle; do nothing. Hardware remains as last set.
                break;

            case IDLE:
            default:
                // Shouldn't be here during active OpMode; remain idle
                break;
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
                            new BezierLine(new Pose(48.000, 96.000), new Pose(44.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 84.000), new Pose(24.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 84.000), new Pose(48.000, 96.000))
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
                            new BezierLine(new Pose(46.000, 57.000), new Pose(15.000, 57.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 57.000), new Pose(48.000, 96.000))
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
}