package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

/**
 * FarmModeBlueAuto — Adapted from BluePedroAuto logic per your instructions:
 *  - Uses provided 7-path set (shoot pose: 52,14)
 *  - Autoshooter RPM set to 140
 *  - Turret logic removed (no TurretController)
 *  - Intake/claw sequencing: applies to paths that start OR end at the shoot pose;
 *    additionally start-intake behavior for path indices as required by the provided paths.
 *
 * New shoot-pose logic (per your request):
 *  - When PRE_ACTION leads to INTAKE_RUN at the shoot pose, the OpMode will:
 *      * Start intake and monitor flywheel RPM.
 *      * If flywheel RPM deviates from the target (AUTO_SHOOTER_RPM +/- tolerance), stop intake
 *        and wait for RPM to return to tolerance.
 *      * Once RPM returns to tolerance, resume intake and require it to run for REQUIRED_STABLE_SECONDS (5s).
 *      * After that stable-run period, perform claw action (close for CLAW_CLOSE_MS, then open),
 *        then continue with next path.
 *    This sequence repeats each time we enter PRE_ACTION (i.e. when reaching the shoot pose).
 *
 *  - For "non-shoot" intake segments (continuous intakes that run during a path), behavior is unchanged:
 *    intake runs while the path is active and is stopped when that path finishes.
 *
 * Notes:
 *  - If Flywheel is not present/mapped, the code falls back to legacy INTAKE_RUN_SECONDS timing.
 *  - Verify hardware names: "shooter", "intakeMotor", "leftCompressionServo", "rightCompressionServo", "clawServo".
 */
@Autonomous(name = "FARMODE BLUE 12 Ball", group = "Autonomous")
@Configurable
public class FarmModeBlueAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, PRE_ACTION, INTAKE_RUN, CLAW_ACTION, FINISHED }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    // Timers
    private Timer intakeTimer;
    private Timer timedIntakeTimer;
    private Timer preActionTimer;
    private Timer poseWaitTimer;
    private Timer shootStableTimer;

    // Timing constants
    private static final double INTAKE_RUN_SECONDS = 2.5; // fallback run for non-shoot segments
    private static final double TIMED_INTAKE_SECONDS = 1.0;
    private static final double PRE_ACTION_WAIT_SECONDS = 0.3;
    private static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;
    private static final long CLAW_CLOSE_MS = 250L;

    // Shooter + rpm/stability settings
    private DcMotor shooterMotor;
    private Flywheel flywheel;
    private static final double AUTO_SHOOTER_RPM = 140.0; // per your request
    private static final double RPM_TOLERANCE = 0.05; // 5% tolerance for "stable" detection
    private static final double REQUIRED_STABLE_SECONDS = 5.0; // run intake for 5s at stable RPM before claw

    // Intake + compression hardware
    private DcMotor intakeMotor;
    private Servo leftCompressionServo;
    private Servo rightCompressionServo;

    // Claw servo
    private Servo clawServo;

    // Intake/compression values
    private static final double INTAKE_ON_POWER = 1.0;
    private static final double LEFT_COMPRESSION_ON = 1.0;
    private static final double RIGHT_COMPRESSION_ON = 0.0;
    private static final double LEFT_COMPRESSION_OFF = 0.5;
    private static final double RIGHT_COMPRESSION_OFF = 0.5;

    // intake segment tracking
    private int intakeSegmentEnd = -1;
    private boolean timedIntakeActive = false;

    // PRE_ACTION flags
    private boolean preActionTimerStarted = false;
    private boolean preActionEntered = false;

    // shoot-pose intake-specific flags
    private boolean inShootPoseSequence = false;     // true when INTAKE_RUN originated from PRE_ACTION at shoot pose
    private boolean waitingForRPMStable = false;     // true when RPM deviated and we stopped intake while at shoot pose

    // Shoot pose constants
    private static final double SHOOT_POSE_X = 52.0;
    private static final double SHOOT_POSE_Y = 14.0;
    private static final double START_POSE_TOLERANCE_IN = 6.0;

    // claw action timestamp
    private long clawActionStartMs = 0L;

    // shooter wait
    private long shooterWaitStartMs = -1;
    private static final long SHOOTER_WAIT_TIMEOUT_MS = 4000L;

    public FarmModeBlueAuto() {}

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Keep same starting pose as your earlier example (adjust if needed)
        follower.setStartingPose(new Pose(63, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        intakeTimer = new Timer();
        timedIntakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        shootStableTimer = new Timer();
        nextPathIndex = -1;
        intakeSegmentEnd = -1;
        preActionTimerStarted = false;
        preActionEntered = false;
        timedIntakeActive = false;
        inShootPoseSequence = false;
        waitingForRPMStable = false;

        // hardware mapping
        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            if (shooterMotor != null) {
                shooterMotor.setDirection(DcMotor.Direction.REVERSE);
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "shooter map failed: " + e.getMessage());
        }

        try {
            if (shooterMotor != null) flywheel = new Flywheel(shooterMotor, telemetry);
            if (flywheel != null) {
                flywheel.setShooterOn(false);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "flywheel creation failed: " + e.getMessage());
        }

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            leftCompressionServo = hardwareMap.get(Servo.class, "leftCompressionServo");
            rightCompressionServo = hardwareMap.get(Servo.class, "rightCompressionServo");
            if (intakeMotor != null) intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            if (intakeMotor != null) intakeMotor.setPower(0.0);
            if (leftCompressionServo != null) leftCompressionServo.setPosition(LEFT_COMPRESSION_OFF);
            if (rightCompressionServo != null) rightCompressionServo.setPosition(RIGHT_COMPRESSION_OFF);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "intake mapping failed: " + e.getMessage());
        }

        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            if (clawServo != null) clawServo.setPosition(0.63);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "claw mapping failed: " + e.getMessage());
        }

        panelsTelemetry.debug("Status", "Initialized (shooter OFF until start)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // warm up
        if (flywheel != null) {
//            flywheel.update(System.currentTimeMillis(), false);
        }
    }

    @Override
    public void start() {
        if (flywheel != null) {
            flywheel.setShooterOn(true);
            flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
        }
        shooterWaitStartMs = System.currentTimeMillis();
        state = AutoState.WAIT_FOR_SHOOTER;
    }

    @Override
    public void loop() {
        follower.update();
        long nowMs = System.currentTimeMillis();

        if (flywheel != null) {
            flywheel.handleLeftTrigger(false);
            flywheel.update(nowMs, false);
        }

        runStateMachine(nowMs);

        // telemetry
        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
        try {
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        } catch (Exception ignore) {}
        if (flywheel != null) {
            panelsTelemetry.debug("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
            panelsTelemetry.debug("Fly Target", String.format("%.1f", flywheel.getTargetRPM()));
            panelsTelemetry.debug("Fly On", flywheel.isShooterOn());
            panelsTelemetry.debug("Fly AtTarget", flywheel.isAtTarget());
        }
        if (intakeMotor != null) {
            panelsTelemetry.debug("Intake Power", intakeMotor.getPower());
            panelsTelemetry.debug("LeftCompPos", leftCompressionServo != null ? leftCompressionServo.getPosition() : -1);
            panelsTelemetry.debug("RightCompPos", rightCompressionServo != null ? rightCompressionServo.getPosition() : -1);
        }
        if (clawServo != null) {
            panelsTelemetry.debug("ClawPos", clawServo.getPosition());
        }

        panelsTelemetry.debug("DistToShootPose", String.format("%.2f", distanceToShootPose()));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (flywheel != null) {
            flywheel.setShooterOn(false);
            flywheel.update(System.currentTimeMillis(), false);
        }
        stopIntake();
        if (clawServo != null) clawServo.setPosition(0.63);
        state = AutoState.FINISHED;
    }

    // intake helpers
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

    // Helper: check which path ends at shoot pose
    private boolean endsAtShoot(int pathIndex) {
        // per provided Paths: Path1, Path3, Path6 end at 52,14
        return pathIndex == 1 || pathIndex == 3 || pathIndex == 6;
    }

    // Helper: check which path starts at shoot pose (we want PRE_ACTION also when next path starts at shoot)
    private boolean startsAtShoot(int pathIndex) {
        // per provided Paths: Path2, Path4, Path7 start at 52,14
        return pathIndex == 2 || pathIndex == 4 || pathIndex == 7;
    }

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

    private void startPath(int idx) {
        if (idx < 1 || idx > 7) {
            currentPathIndex = 0;
            state = AutoState.FINISHED;
            return;
        }

        // For the provided route: start intake segments for certain indices.
        // Per your requests, we start intake when path is one of those that should run intake while active.
        // We'll enable continuous intake for paths that semantically correspond to gathering:
        // choose Path1 and Path5 as explicitly requested and Path3/6 as end-of-run gatherers as previously.
        if (idx == 1 || idx == 3 || idx == 5 || idx == 6) {
            intakeSegmentEnd = idx;
            startIntake();
        }

        // Also start a timed short intake on some path starts if desired (kept as convenience)
        if (idx == 4) {
            startIntake();
            timedIntakeTimer.resetTimer();
            timedIntakeActive = true;
            panelsTelemetry.debug("TimedIntake", "Started timed intake for path " + idx);
        }

        switch (idx) {
            case 1: follower.followPath(paths.Path1); break;
            case 2: follower.followPath(paths.Path2); break;
            case 3: follower.followPath(paths.Path3); break;
            case 4: follower.followPath(paths.Path4); break;
            case 5: follower.followPath(paths.Path5); break;
            case 6: follower.followPath(paths.Path6); break;
            case 7: follower.followPath(paths.Path7); break;
            default: break;
        }

        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
    }

    private void runStateMachine(long nowMs) {
        // handle timed intake expiration (independent)
        if (timedIntakeActive) {
            if (timedIntakeTimer.getElapsedTimeSeconds() >= TIMED_INTAKE_SECONDS) {
                stopIntake();
                timedIntakeActive = false;
                intakeSegmentEnd = -1;
                panelsTelemetry.debug("TimedIntake", "Timed intake ended");
            } else {
                panelsTelemetry.debug("TimedIntake", String.format("remaining=%.2fs", TIMED_INTAKE_SECONDS - timedIntakeTimer.getElapsedTimeSeconds()));
            }
        }

        switch (state) {
            case WAIT_FOR_SHOOTER: {
                boolean atTarget = (flywheel != null && flywheel.isAtTarget());
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (System.currentTimeMillis() - shooterWaitStartMs);
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) {
                    startPath(1);
                }
                break;
            }

            case RUNNING_PATH:
                if (!follower.isBusy()) {
                    int finished = currentPathIndex;

                    // stop intake for continuous-intake segments
                    if (intakeSegmentEnd == finished) {
                        stopIntake();
                        intakeSegmentEnd = -1;
                    }

                    int next = finished + 1;
                    if (next > 7) {
                        state = AutoState.FINISHED;
                    } else {
                        // If finished path ended at shoot OR the next path starts at shoot, enter PRE_ACTION
                        if (endsAtShoot(finished) || startsAtShoot(next)) {
                            nextPathIndex = next;
                            preActionTimerStarted = false;
                            preActionEntered = false;
                            state = AutoState.PRE_ACTION;
                        } else {
                            startPath(next);
                        }
                    }
                }
                break;

            case PRE_ACTION:
                if (!preActionEntered) {
                    poseWaitTimer.resetTimer();
                    preActionTimerStarted = false;
                    preActionEntered = true;
                    panelsTelemetry.debug("PRE_ACTION", "Entered PRE_ACTION");
                }

                if (!preActionTimerStarted) {
                    double dist = distanceToShootPose();
                    if (dist <= START_POSE_TOLERANCE_IN) {
                        preActionTimer.resetTimer();
                        preActionTimerStarted = true;
                        panelsTelemetry.debug("PRE_ACTION", "At pose: starting PRE_ACTION timer");
                    } else if (poseWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_MAX_POSE_WAIT_SECONDS) {
                        preActionTimer.resetTimer();
                        preActionTimerStarted = true;
                        panelsTelemetry.debug("PRE_ACTION", "Pose-wait timeout: starting PRE_ACTION timer anyway (dist=" + String.format("%.2f", dist) + ")");
                    } else {
                        panelsTelemetry.debug("PRE_ACTION", "Waiting for pose (dist=" + String.format("%.2f", dist) + ")");
                    }
                } else {
                    if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                        // START shoot-pose intake sequence and go to INTAKE_RUN in shoot mode
                        startIntake();
                        intakeTimer.resetTimer();
                        // Mark that this INTAKE_RUN originates from a shoot-pose PRE_ACTION
                        inShootPoseSequence = true;
                        waitingForRPMStable = false;
                        // initialize shootStableTimer depending on current RPM
                        if (flywheel != null && Math.abs(flywheel.getCurrentRPM() - AUTO_SHOOTER_RPM) <= (RPM_TOLERANCE * AUTO_SHOOTER_RPM)) {
                            shootStableTimer.resetTimer();
                        } else {
                            // not stable yet; we'll wait for it to become stable and start the stable timer then
                            shootStableTimer.resetTimer();
                            waitingForRPMStable = true;
                            stopIntake(); // stop until RPM returns
                        }
                        state = AutoState.INTAKE_RUN;
                    }
                }
                break;

            case INTAKE_RUN:
                if (inShootPoseSequence) {
                    // New logic that depends on flywheel RPM stability
                    if (flywheel == null) {
                        // fallback: no flywheel available, behave like legacy INTAKE_RUN_SECONDS
                        if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                            // perform claw sequence
                            if (intakeSegmentEnd == -1) stopIntake();
                            if (flywheel != null) flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                            if (clawServo != null) clawServo.setPosition(0.2);
                            clawActionStartMs = System.currentTimeMillis();
                            // reset flags
                            inShootPoseSequence = false;
                            waitingForRPMStable = false;
                            state = AutoState.CLAW_ACTION;
                        }
                    } else {
                        double currentRPM = flywheel.getCurrentRPM();
                        boolean rpmStable = Math.abs(currentRPM - AUTO_SHOOTER_RPM) <= (RPM_TOLERANCE * AUTO_SHOOTER_RPM);

                        if (!rpmStable) {
                            // RPM changed away from target: stop intake and wait for it to return
                            if (!waitingForRPMStable) {
                                panelsTelemetry.debug("ShootSeq", "RPM deviated (now=" + String.format("%.1f", currentRPM) + "), stopping intake and waiting");
                            }
                            waitingForRPMStable = true;
                            stopIntake();
                            // do not progress to claw until RPM returns and stable timer completes
                        } else {
                            // RPM is stable
                            if (waitingForRPMStable) {
                                // we just regained stable RPM; restart intake and reset stable timer
                                panelsTelemetry.debug("ShootSeq", "RPM returned to target (now=" + String.format("%.1f", currentRPM) + "), restarting stable timer");
                                startIntake();
                                shootStableTimer.resetTimer();
                                waitingForRPMStable = false;
                            } else {
                                // if intake isn't running, ensure it's running
                                if (intakeMotor != null && intakeMotor.getPower() == 0.0) {
                                    startIntake();
                                }
                                // If stable timer has run long enough, perform claw action
                                if (shootStableTimer.getElapsedTimeSeconds() >= REQUIRED_STABLE_SECONDS) {
                                    // perform claw action
                                    if (intakeSegmentEnd == -1) stopIntake();
                                    flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                                    if (clawServo != null) clawServo.setPosition(0.2); // close
                                    clawActionStartMs = System.currentTimeMillis();
                                    // reset shoot-pose flags
                                    inShootPoseSequence = false;
                                    waitingForRPMStable = false;
                                    state = AutoState.CLAW_ACTION;
                                } else {
                                    panelsTelemetry.debug("ShootSeq", String.format("stableRun=%.2fs/%.2fs", shootStableTimer.getElapsedTimeSeconds(), REQUIRED_STABLE_SECONDS));
                                }
                            }
                        }
                    }
                } else {
                    // Non-shoot INTAKE_RUN: legacy timed behavior
                    if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                        if (intakeSegmentEnd == -1) stopIntake();
                        if (flywheel != null) flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                        if (clawServo != null) clawServo.setPosition(0.2); // close
                        clawActionStartMs = System.currentTimeMillis();
                        state = AutoState.CLAW_ACTION;
                    }
                }
                break;

            case CLAW_ACTION:
                if (System.currentTimeMillis() >= clawActionStartMs + CLAW_CLOSE_MS) {
                    // open claw and continue
                    if (clawServo != null) clawServo.setPosition(0.63);
                    if (nextPathIndex > 0 && nextPathIndex <= 7) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;

            case FINISHED:
                // do nothing
                break;

            case IDLE:
            default:
                break;
        }
    }

    // Paths per your latest coordinates (7 paths)
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(63.000, 8.000), new Pose(52.000, 14.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(109))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.000, 14.000), new Pose(20.000, 14.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 14.000), new Pose(52.000, 14.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(109))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.000, 14.000), new Pose(46.000, 32.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(46.000, 32.000), new Pose(20.000, 32.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 32.000), new Pose(52.000, 14.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(109))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(52.000, 14.000), new Pose(46.000, 25.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(109))
                    .build();
        }
    }
}