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
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;

/**
 * FarModeRedAuto — mirrored from FarModeBlueAuto (Y mirrored, headings negated).
 */
@Autonomous(name = "FARMODE RED 🔴", group = "Autonomous", preselectTeleOp = "HORS EXPERIMENTAL 🤖")
@Configurable
public class FarModeRedAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState {
        IDLE,
        WAIT_FOR_SHOOTER,
        RUNNING_PATH,
        PRE_ACTION,
        INTAKE_RUN,
        CLAW_ACTION,
        WAIT_NO_MOVEMENT,
        FINISHED
    }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private Timer timedIntakeTimer;
    private Timer preActionTimer;
    private Timer poseWaitTimer;
    private Timer shootStableTimer;
    private Timer waitTimer;

    private static final double INTAKE_RUN_SECONDS = 2.5;
    private static final double TIMED_INTAKE_SECONDS = 1.0;
    private static final double PRE_ACTION_WAIT_SECONDS = 0.3;
    private static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;
    private static final long CLAW_CLOSE_MS = 250L;

    public static double WAIT_TIME_NO_MOVEMENT = 2.0; // seconds

    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private FlywheelController flywheel;
    private static final double AUTO_SHOOTER_RPM = FlywheelController.TARGET_RPM_FAR;
    private static final double RPM_TOLERANCE = 0.05;
    private static final double REQUIRED_STABLE_SECONDS = 3.0;

    private DcMotor intakeMotor;
    private Servo leftCompressionServo;
    private Servo rightCompressionServo;

    private Servo clawServo;

    private static final double INTAKE_ON_POWER = 1.0;
    private static final double LEFT_COMPRESSION_ON = 1.0;
    private static final double RIGHT_COMPRESSION_ON = 0.0;
    private static final double LEFT_COMPRESSION_OFF = 0.5;
    private static final double RIGHT_COMPRESSION_OFF = 0.5;

    private int intakeSegmentEnd = -1;
    private boolean timedIntakeActive = false;

    private boolean preActionTimerStarted = false;
    private boolean preActionEntered = false;

    private boolean inShootPoseSequence = false;
    private boolean waitingForRPMStable = false;

    // Mirrored shoot pose (Y mirrored from 14 -> 130)
    private static final double SHOOT_POSE_X = 52.0;
    private static final double SHOOT_POSE_Y = 130.0;
    private static final double START_POSE_TOLERANCE_IN = 6.0;

    private long clawActionStartMs = 0L;

    private long shooterWaitStartMs = -1;
    private static final long SHOOTER_WAIT_TIMEOUT_MS = 4000L;

    private BNO055IMU pinpointImu = null;
    private BNO055IMU hubImu = null;
    private BNO055IMU imu = null;

    public FarModeRedAuto() {}

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Mirrored start pose: (63, 8, 90) -> (63, 136, -90)
        follower.setStartingPose(new Pose(63, 136, Math.toRadians(-90)));

        paths = new Paths(follower);

        intakeTimer = new Timer();
        timedIntakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        shootStableTimer = new Timer();
        waitTimer = new Timer();

        nextPathIndex = -1;
        intakeSegmentEnd = -1;
        preActionTimerStarted = false;
        preActionEntered = false;
        timedIntakeActive = false;
        inShootPoseSequence = false;
        waitingForRPMStable = false;

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
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
            if (shooterMotor2 != null) {
                shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
                shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "shooter2 map failed: " + e.getMessage());
        }

        try {
            if (shooterMotor != null) flywheel = new FlywheelController(shooterMotor, shooterMotor2, telemetry);
            if (flywheel != null) flywheel.setShooterOn(false);
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

        try {
            try {
                pinpointImu = hardwareMap.get(BNO055IMU.class, "pinpoint");
                panelsTelemetry.debug("Init", "PinPoint IMU 'pinpoint' found");
            } catch (Exception e) {
                pinpointImu = null;
            }

            try {
                hubImu = hardwareMap.get(BNO055IMU.class, "imu");
                panelsTelemetry.debug("Init", "Expansion Hub IMU 'imu' found");
            } catch (Exception e) {
                hubImu = null;
            }

            imu = (pinpointImu != null) ? pinpointImu : hubImu;

            if (imu != null) {
                BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
                imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                imu.initialize(imuParams);
            } else {
                panelsTelemetry.debug("Init", "IMU not found or failed to init");
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "IMU init error: " + e.getMessage());
        }

        panelsTelemetry.debug("Status", "Initialized (shooter OFF until start)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (flywheel != null) {
            // flywheel.update(System.currentTimeMillis(), false);
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

        if (shooterMotor != null && shooterMotor2 != null) {
            try {
                shooterMotor2.setPower(shooterMotor.getPower());
            } catch (Exception e) {
                panelsTelemetry.debug("Shooter2", "Power mirror error: " + e.getMessage());
            }
        }

        runStateMachine(nowMs);

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

    private boolean endsAtShoot(int pathIndex) {
        return pathIndex == 1 || pathIndex == 3 || pathIndex == 6;
    }

    private boolean startsAtShoot(int pathIndex) {
        return pathIndex == 2 || pathIndex == 4 || pathIndex == 7;
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

        if (idx == 1 || idx == 3 || idx == 5 || idx == 6) {
            intakeSegmentEnd = idx;
            startIntake();
        }

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

                    if (intakeSegmentEnd == finished) {
                        stopIntake();
                        intakeSegmentEnd = -1;
                    }

                    if (finished == 3) {
                        nextPathIndex = finished + 1;
                        waitTimer.resetTimer();
                        if (flywheel != null) {
                            flywheel.setTargetRPM(FlywheelController.TARGET_RPM_CLOSE);
                        }
                        panelsTelemetry.debug("WAIT_NO_MOVEMENT", "Entered after gateClear; waiting " + WAIT_TIME_NO_MOVEMENT + "s with close-mode RPM");
                        state = AutoState.WAIT_NO_MOVEMENT;
                        break;
                    }

                    int next = finished + 1;
                    if (next > 7) {
                        state = AutoState.FINISHED;
                    } else {
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
                        startIntake();
                        intakeTimer.resetTimer();
                        inShootPoseSequence = true;
                        waitingForRPMStable = false;
                        if (flywheel != null && Math.abs(flywheel.getCurrentRPM() - AUTO_SHOOTER_RPM) <= (RPM_TOLERANCE * AUTO_SHOOTER_RPM)) {
                            shootStableTimer.resetTimer();
                        } else {
                            shootStableTimer.resetTimer();
                            waitingForRPMStable = true;
                            stopIntake();
                        }
                        state = AutoState.INTAKE_RUN;
                    }
                }
                break;

            case INTAKE_RUN:
                if (inShootPoseSequence) {
                    if (flywheel == null) {
                        if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                            if (intakeSegmentEnd == -1) stopIntake();
                            if (flywheel != null) flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                            if (clawServo != null) clawServo.setPosition(0.2);
                            clawActionStartMs = System.currentTimeMillis();
                            inShootPoseSequence = false;
                            waitingForRPMStable = false;
                            state = AutoState.CLAW_ACTION;
                        }
                    } else {
                        double currentRPM = flywheel.getCurrentRPM();
                        boolean rpmStable = Math.abs(currentRPM - AUTO_SHOOTER_RPM) <= (RPM_TOLERANCE * AUTO_SHOOTER_RPM);

                        if (!rpmStable) {
                            if (!waitingForRPMStable) {
                                panelsTelemetry.debug("ShootSeq", "RPM deviated (now=" + String.format("%.1f", currentRPM) + "), stopping intake and waiting");
                            }
                            waitingForRPMStable = true;
                            stopIntake();
                        } else {
                            if (waitingForRPMStable) {
                                panelsTelemetry.debug("ShootSeq", "RPM returned to target (now=" + String.format("%.1f", currentRPM) + "), restarting stable timer");
                                startIntake();
                                shootStableTimer.resetTimer();
                                waitingForRPMStable = false;
                            } else {
                                if (intakeMotor != null && intakeMotor.getPower() == 0.0) {
                                    startIntake();
                                }
                                if (shootStableTimer.getElapsedTimeSeconds() >= REQUIRED_STABLE_SECONDS) {
                                    if (intakeSegmentEnd == -1) stopIntake();
                                    flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                                    if (clawServo != null) clawServo.setPosition(0.2);
                                    clawActionStartMs = System.currentTimeMillis();
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
                    if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                        if (intakeSegmentEnd == -1) stopIntake();
                        if (flywheel != null) flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                        if (clawServo != null) clawServo.setPosition(0.2);
                        clawActionStartMs = System.currentTimeMillis();
                        state = AutoState.CLAW_ACTION;
                    }
                }
                break;

            case CLAW_ACTION:
                if (System.currentTimeMillis() >= clawActionStartMs + CLAW_CLOSE_MS) {
                    if (clawServo != null) clawServo.setPosition(0.63);
                    if (nextPathIndex > 0 && nextPathIndex <= 7) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;

            case WAIT_NO_MOVEMENT:
                panelsTelemetry.debug("WAIT_NO_MOVEMENT", String.format("elapsed=%.2fs/%.2fs", waitTimer.getElapsedTimeSeconds(), WAIT_TIME_NO_MOVEMENT));
                if (waitTimer.getElapsedTimeSeconds() >= WAIT_TIME_NO_MOVEMENT) {
                    int toStart = nextPathIndex;
                    nextPathIndex = -1;
                    if (toStart > 0 && toStart <= 7) {
                        if (flywheel != null) flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
                        startPath(toStart);
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;

            case FINISHED:
                break;

            case IDLE:
            default:
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

        public Paths(Follower follower) {
            // Mirrored paths (Y mirrored from blue: y' = 144 - y, headings negated)
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(63.000, 136.000), new Pose(52.000, 130.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-109))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(52.000, 130.000), new Pose(20.000, 130.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-109), Math.toRadians(-180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 130.000), new Pose(52.000, 130.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-109))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(52.000, 130.000), new Pose(46.000, 112.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-109), Math.toRadians(-180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(46.000, 112.000), new Pose(20.000, 112.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 112.000), new Pose(52.000, 130.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-109))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(52.000, 130.000), new Pose(46.000, 119.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(-109))
                    .build();
        }
    }
}