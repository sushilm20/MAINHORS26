package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.subsystems.IntakeBallDetector;
import org.firstinspires.ftc.teamcode.tracking.BearingTurretController;

@Autonomous(name = "Blue 15 Ball 🔷", group = "Autonomous", preselectTeleOp = "A HORS OFFICIAL ⭐")
@Configurable
public class Blue15Ball extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState {
        IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, WAIT_FIRST_SHOOT,
        CLOSED_INTAKE_SEQUENCE, PRE_ACTION, INTAKE_RUN, CLAW_ACTION, FINISHED
    }

    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private Timer timedIntakeTimer;
    private Timer preActionTimer;
    private Timer firstShootWaitTimer;
    private Timer speedStableTimer;

    private boolean timedIntakeActive = false;
    private boolean preActionTimerStarted = false;
    private boolean preActionEntered = false;
    private boolean speedStableTimerStarted = false;

    private long clawActionStartMs = 0L;
    private long shooterWaitStartMs = -1;
    private long autoStartMs = -1;
    private boolean shutdownDone = false;

    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private DcMotor turretMotor;
    private DcMotorEx intakeMotor;

    private Servo clawServo;
    private Servo rightHoodServo;
    private Servo gateServo;
    private boolean gateClosed = true;

    private BNO055IMU pinpointImu = null;
    private BNO055IMU hubImu = null;
    private BNO055IMU imu = null;

    private FlywheelController flywheel;
    private BearingTurretController bearingTurretController;
    private boolean turretTrackingEnabled = false;

    private static final double AUTO_SHOOTER_RPM = 2400;

    // Velocity tracking
    private double lastPoseX = Double.NaN;
    private double lastPoseY = Double.NaN;
    private double lastPoseHeadingRad = Double.NaN;
    private long lastPoseTimeMs = -1L;
    private double translationalSpeedInPerS = 0.0;
    private double angularSpeedDegPerS = 0.0;

    // ====== Tunables ======
    @Sorter(sort = 0)  public static double INTAKE_RUN_SECONDS = 0.30;
    @Sorter(sort = 1)  public static double TIMED_INTAKE_SECONDS = 1.0;
    @Sorter(sort = 2)  public static double PRE_ACTION_FIRST_SHOOT_WAIT_SECONDS = 0.8;
    @Sorter(sort = 3)  public static double PRE_ACTION_WAIT_SECONDS = 1.0;
    @Sorter(sort = 4)  public static long SHOOTER_WAIT_TIMEOUT_MS = 1100L;

    @Sorter(sort = 10) public static double INTAKE_ON_POWER = -1.0;
    @Sorter(sort = 11) public static double SHOOT_POSE_INTAKE_POWER = -1.0;
    @Sorter(sort = 12) public static double CLOSED_INTAKE_POWER = -0.67;
    @Sorter(sort = 13) public static double INTAKE_HOLD_POWER = -0.5;

    @Sorter(sort = 20) public static double SHOOT_MAX_TRANSLATIONAL_SPEED_IN_PER_S = 1.5;
    @Sorter(sort = 21) public static double SHOOT_MAX_ANGULAR_SPEED_DEG_PER_S = 20.0;
    @Sorter(sort = 22) public static double SPEED_STABLE_HOLD_SECONDS = 0.15;

    @Sorter(sort = 30) public static double GATE_OPEN = 0.67;
    @Sorter(sort = 31) public static double GATE_CLOSED = 0.485;
    @Sorter(sort = 32) public static double GATE_OPEN_TOLERANCE_IN = 2.2;
    @Sorter(sort = 33) public static double GATE_CLOSE_TOLERANCE_IN = 4.0;

    // Gate opens only when robot is settled near shoot
    @Sorter(sort = 34) public static double GATE_OPEN_MAX_TRANS_SPEED = 1.1;
    @Sorter(sort = 35) public static double GATE_OPEN_MAX_ANG_SPEED = 14.0;

    // Path slowdown tunables
    @Sorter(sort = 40) public static double SLOW_T_VALUE_CONSTRAINT = 0.995;
    @Sorter(sort = 41) public static double SLOW_TIMEOUT_MS = 180;
    @Sorter(sort = 42) public static double SLOW_BRAKING_STRENGTH = 0.70;
    @Sorter(sort = 43) public static double SLOW_BRAKING_START = 0.84;
    @Sorter(sort = 44) public static double SLOW_VELOCITY_CONSTRAINT = 0.10;
    @Sorter(sort = 45) public static double SLOW_TRANSLATIONAL_CONSTRAINT = 0.10;
    @Sorter(sort = 46) public static double SLOW_HEADING_CONSTRAINT_RAD = 0.007;

    // Shoot pose for gate logic
    public static double SHOOT_POSE_X = 54.0;
    public static double SHOOT_POSE_Y = 84.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.661, 122.000, Math.toRadians(142.5)));
        paths = new Paths(follower);

        intakeTimer = new Timer();
        timedIntakeTimer = new Timer();
        preActionTimer = new Timer();
        firstShootWaitTimer = new Timer();
        speedStableTimer = new Timer();

        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
            turretMotor = hardwareMap.get(DcMotor.class, "turret");

            if (shooterMotor != null) {
                shooterMotor.setDirection(DcMotor.Direction.REVERSE);
                shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (shooterMotor2 != null) {
                shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
                shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (turretMotor != null) {
                turretMotor.setDirection(DcMotor.Direction.FORWARD);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turretMotor.setPower(0.0); // no premove in init
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Motor map error: " + e.getMessage());
        }

        try {
            try { pinpointImu = hardwareMap.get(BNO055IMU.class, "pinpoint"); }
            catch (Exception ignored) { pinpointImu = null; }

            imu = (pinpointImu != null) ? pinpointImu : hubImu;
            if (imu != null) {
                BNO055IMU.Parameters p = new BNO055IMU.Parameters();
                p.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                p.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                imu.initialize(p);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "IMU error: " + e.getMessage());
        }

        try {
            if (shooterMotor != null) {
                flywheel = new FlywheelController(shooterMotor, shooterMotor2, telemetry);
                flywheel.setShooterOn(false);
            }
            if (turretMotor != null) {
                bearingTurretController = new BearingTurretController(turretMotor, follower, telemetry);
                bearingTurretController.zeroEncoder();
                bearingTurretController.clearPid();
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Controller init error: " + e.getMessage());
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake map error: " + e.getMessage());
        }

        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
        } catch (Exception ignored) {}

        try {
            rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
            if (rightHoodServo != null) rightHoodServo.setPosition(0.16);
        } catch (Exception ignored) {}

        try {
            gateServo = hardwareMap.get(Servo.class, "gateServo");
            if (gateServo != null) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
            }
        } catch (Exception ignored) {}

        turretTrackingEnabled = false;
        panelsTelemetry.debug("Status", "Initialized (turret BRAKE only, no tracking)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (turretMotor != null) turretMotor.setPower(0.0);
    }

    @Override
    public void start() {
        autoStartMs = System.currentTimeMillis();
        turretTrackingEnabled = true;

        if (flywheel != null) {
            flywheel.setShooterOn(true);
            flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
        }
        if (bearingTurretController != null) {
            bearingTurretController.clearPid();
        }

        shooterWaitStartMs = System.currentTimeMillis();
        state = AutoState.WAIT_FOR_SHOOTER;
    }

    @Override
    public void loop() {
        follower.update();
        long nowMs = System.currentTimeMillis();
        updateChassisSpeeds(nowMs);

        if (flywheel != null) {
            flywheel.handleLeftTrigger(false);
            flywheel.update(nowMs, false);
        }

        if (bearingTurretController != null) {
            if (turretTrackingEnabled) bearingTurretController.update(false, 0.0);
            else if (turretMotor != null) turretMotor.setPower(0.0);
        }

        runStateMachine();

        if (state != AutoState.WAIT_FIRST_SHOOT) updateGate();

        if (state != AutoState.IDLE && state != AutoState.FINISHED) {
            if (intakeMotor != null && intakeMotor.getPower() == 0.0) intakeMotor.setPower(INTAKE_HOLD_POWER);
        }

        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Vtrans", String.format("%.2f", translationalSpeedInPerS));
        panelsTelemetry.debug("Vang", String.format("%.2f", angularSpeedDegPerS));
        panelsTelemetry.debug("GateClosed", gateClosed);
        if (turretMotor != null) panelsTelemetry.debug("TurretEnc", turretMotor.getCurrentPosition());
        if (intakeMotor != null) panelsTelemetry.debug("IntakePower", intakeMotor.getPower());
        panelsTelemetry.update(telemetry);

        if (state == AutoState.FINISHED && !shutdownDone) {
            resetToInitState();
            shutdownDone = true;
        }
    }

    private void runStateMachine() {
        if (timedIntakeActive && timedIntakeTimer.getElapsedTimeSeconds() >= TIMED_INTAKE_SECONDS) {
            startIntake(INTAKE_ON_POWER);
            timedIntakeActive = false;
        }

        switch (state) {
            case WAIT_FOR_SHOOTER: {
                boolean atTarget = (flywheel != null && flywheel.isAtTarget());
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (System.currentTimeMillis() - shooterWaitStartMs);
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) startPath(1);
                break;
            }

            case RUNNING_PATH: {
                if (!follower.isBusy()) {
                    int finished = currentPathIndex;

                    if (isShootPath(finished)) {
                        nextPathIndex = finished + 1;
                        preActionEntered = false;
                        preActionTimerStarted = false;

                        if (finished == 1) {
                            stopIntake();
                            forceGateClosed();
                            firstShootWaitTimer.resetTimer();
                            state = AutoState.WAIT_FIRST_SHOOT;
                        } else {
                            state = AutoState.CLOSED_INTAKE_SEQUENCE;
                        }
                    } else {
                        int next = finished + 1;
                        if (next > 14) state = AutoState.FINISHED;
                        else startPath(next);
                    }
                }
                break;
            }

            case WAIT_FIRST_SHOOT:
                if (firstShootWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_FIRST_SHOOT_WAIT_SECONDS) {
                    state = AutoState.CLOSED_INTAKE_SEQUENCE;
                }
                break;

            case CLOSED_INTAKE_SEQUENCE:
                if (isShootMotionStable()) {
                    startIntake(CLOSED_INTAKE_POWER);

                    if (!speedStableTimerStarted) {
                        speedStableTimer.resetTimer();
                        speedStableTimerStarted = true;
                    }

                    if (speedStableTimer.getElapsedTimeSeconds() >= SPEED_STABLE_HOLD_SECONDS) {
                        speedStableTimerStarted = false;

                        // ── Now that intake is slowed & stable, reliably check for balls ──
                        if (currentPathIndex != 1 && !IntakeBallDetector.hasBalls(intakeMotor)) {
                            panelsTelemetry.debug("SKIP_SHOOT", "Path " + currentPathIndex + " has 0 balls – skipping shoot");
                            if (nextPathIndex > 0 && nextPathIndex <= 14) {
                                startPath(nextPathIndex);
                                nextPathIndex = -1;
                            } else {
                                state = AutoState.FINISHED;
                            }
                        } else {
                            state = AutoState.PRE_ACTION;
                        }
                    }
                } else {
                    speedStableTimerStarted = false;
                }
                break;

            case PRE_ACTION:
                if (!preActionEntered) {
                    preActionTimer.resetTimer();
                    preActionTimerStarted = true;
                    preActionEntered = true;
                }
                if (preActionTimerStarted && preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                    startIntake(SHOOT_POSE_INTAKE_POWER);
                    intakeTimer.resetTimer();
                    state = AutoState.INTAKE_RUN;
                }
                break;

            case INTAKE_RUN:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                    startIntake(INTAKE_ON_POWER);
                    if (flywheel != null) flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                    if (clawServo != null) clawServo.setPosition(ClawController.CLAW_CLOSED);
                    clawActionStartMs = System.currentTimeMillis();
                    state = AutoState.CLAW_ACTION;
                }
                break;

            case CLAW_ACTION:
                if (System.currentTimeMillis() >= clawActionStartMs + ClawController.CLAW_CLOSE_MS) {
                    if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
                    if (nextPathIndex > 0 && nextPathIndex <= 14) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;

            default:
                break;
        }
    }

    private void startPath(int idx) {
        if (idx < 1 || idx > 14) {
            state = AutoState.FINISHED;
            return;
        }

        if (isIntakePath(idx)) startIntake(INTAKE_ON_POWER);
        else stopIntake();

        if (isTimedIntakePath(idx)) {
            timedIntakeTimer.resetTimer();
            timedIntakeActive = true;
        }

        switch (idx) {
            case 1:  follower.followPath(paths.startToShoot); break;
            case 2:  follower.followPath(paths.collectFirst3); break;
            case 3:  follower.followPath(paths.backToShootFirst3); break;
            case 4:  follower.followPath(paths.gateOpen1); break;
            case 5:  follower.followPath(paths.gateMoveCollect1); break;
            case 6:  follower.followPath(paths.gateExtendedCollect1); break;
            case 7:  follower.followPath(paths.backToShootSecond3); break;
            case 8:  follower.followPath(paths.gateOpen2); break;
            case 9:  follower.followPath(paths.gateMoveCollect2); break;
            case 10: follower.followPath(paths.gateExtendedCollect2); break;
            case 11: follower.followPath(paths.backToShootThird3); break;
            case 12: follower.followPath(paths.collectForth3); break;
            case 13: follower.followPath(paths.backToShootForth3); break;
            case 14: follower.followPath(paths.path14); break;
        }

        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
    }

    private boolean isShootPath(int idx) {
        return idx == 1 || idx == 3 || idx == 7 || idx == 11 || idx == 13;
    }

    private boolean isIntakePath(int idx) {
        return idx == 2 || idx == 4 || idx == 5 || idx == 6 || idx == 8 || idx == 9 || idx == 10 || idx == 12;
    }

    private boolean isTimedIntakePath(int idx) {
        return idx == 3 || idx == 7 || idx == 11 || idx == 13;
    }

    private void startIntake(double p) { if (intakeMotor != null) intakeMotor.setPower(p); }
    private void stopIntake() { if (intakeMotor != null) intakeMotor.setPower(INTAKE_HOLD_POWER); }
    private void fullStopIntake() { if (intakeMotor != null) intakeMotor.setPower(0.0); }

    private void forceGateClosed() {
        if (gateServo != null && !gateClosed) {
            gateServo.setPosition(GATE_CLOSED);
            gateClosed = true;
        }
    }

    private void updateGate() {
        if (gateServo == null) return;
        double dist = distanceToShootPose();

        boolean slowEnoughForGate = translationalSpeedInPerS <= GATE_OPEN_MAX_TRANS_SPEED
                && angularSpeedDegPerS <= GATE_OPEN_MAX_ANG_SPEED;

        if (dist <= GATE_OPEN_TOLERANCE_IN && slowEnoughForGate && gateClosed) {
            gateServo.setPosition(GATE_OPEN);
            gateClosed = false;
        } else if ((dist >= GATE_CLOSE_TOLERANCE_IN || !slowEnoughForGate) && !gateClosed) {
            gateServo.setPosition(GATE_CLOSED);
            gateClosed = true;
        }
    }

    private double distanceToShootPose() {
        try {
            Pose p = follower.getPose();
            return Math.hypot(p.getX() - SHOOT_POSE_X, p.getY() - SHOOT_POSE_Y);
        } catch (Exception e) {
            return Double.POSITIVE_INFINITY;
        }
    }

    private void updateChassisSpeeds(long nowMs) {
        try {
            Pose p = follower.getPose();
            if (p == null) return;

            double x = p.getX(), y = p.getY(), h = p.getHeading();

            if (lastPoseTimeMs > 0 && !Double.isNaN(lastPoseX) && !Double.isNaN(lastPoseY) && !Double.isNaN(lastPoseHeadingRad)) {
                double dt = Math.max(1, nowMs - lastPoseTimeMs) / 1000.0;
                translationalSpeedInPerS = Math.hypot(x - lastPoseX, y - lastPoseY) / dt;
                angularSpeedDegPerS = Math.toDegrees(Math.abs(normalizeAngleRad(h - lastPoseHeadingRad))) / dt;
            }

            lastPoseX = x; lastPoseY = y; lastPoseHeadingRad = h; lastPoseTimeMs = nowMs;
        } catch (Exception ignored) {}
    }

    private boolean isShootMotionStable() {
        return translationalSpeedInPerS <= SHOOT_MAX_TRANSLATIONAL_SPEED_IN_PER_S
                && angularSpeedDegPerS <= SHOOT_MAX_ANGULAR_SPEED_DEG_PER_S;
    }

    private static double normalizeAngleRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private void resetToInitState() {
        turretTrackingEnabled = false;

        if (flywheel != null) {
            flywheel.setShooterOn(false);
            flywheel.setTargetRPM(0.0);
            flywheel.update(System.currentTimeMillis(), false);
        }
        fullStopIntake();

        if (gateServo != null) {
            gateServo.setPosition(GATE_CLOSED);
            gateClosed = true;
        }
        if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
        if (rightHoodServo != null) rightHoodServo.setPosition(0.16);

        if (turretMotor != null) {
            try { turretMotor.setPower(0.0); } catch (Exception ignored) {}
        }
    }

    @Override
    public void stop() {
        resetToInitState();
        state = AutoState.FINISHED;
    }

    // ================= Paths with per-path slowdown =================
    public static class Paths {
        public PathChain startToShoot, collectFirst3, backToShootFirst3, gateOpen1, gateMoveCollect1, gateExtendedCollect1,
                backToShootSecond3, gateOpen2, gateMoveCollect2, gateExtendedCollect2,
                backToShootThird3, collectForth3, backToShootForth3, path14;

        private static PathConstraints slowEndConstraints() {
            return new PathConstraints(
                    SLOW_T_VALUE_CONSTRAINT,
                    SLOW_VELOCITY_CONSTRAINT,
                    SLOW_TRANSLATIONAL_CONSTRAINT,
                    SLOW_HEADING_CONSTRAINT_RAD,
                    SLOW_TIMEOUT_MS,
                    SLOW_BRAKING_STRENGTH,
                    10,
                    SLOW_BRAKING_START
            );
        }

        public Paths(Follower follower) {
            PathConstraints slow = slowEndConstraints();

            startToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.661, 122.000), new Pose(54.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(142.5), Math.toRadians(180))
                    .setConstraintsForLast(slow)
                    .setBrakingStrength(SLOW_BRAKING_STRENGTH)
                    .setBrakingStart(SLOW_BRAKING_START)
                    .setGlobalDeceleration(SLOW_BRAKING_START)
                    .build();

            collectFirst3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(54.000, 84.000), new Pose(68.000, 56.000), new Pose(13.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            backToShootFirst3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(13.000, 60.000), new Pose(43.000, 63.000), new Pose(54.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setConstraintsForLast(slow)
                    .setBrakingStrength(SLOW_BRAKING_STRENGTH)
                    .setBrakingStart(SLOW_BRAKING_START)
                    .setGlobalDeceleration(SLOW_BRAKING_START)
                    .build();

            gateOpen1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(54.000, 84.000), new Pose(16.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .setConstraintsForLast(slow)
                    .setBrakingStrength(SLOW_BRAKING_STRENGTH)
                    .setBrakingStart(SLOW_BRAKING_START)
                    .setGlobalDeceleration(SLOW_BRAKING_START)
                    .build();

            gateMoveCollect1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(16.000, 60.000), new Pose(13.000, 50.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(120))
                    .build();

            gateExtendedCollect1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(13.000, 50.000), new Pose(13.000, 55.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(135))
                    .build();

            backToShootSecond3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(13.000, 55.000), new Pose(40.000, 54.000), new Pose(54.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .setConstraintsForLast(slow)
                    .setBrakingStrength(SLOW_BRAKING_STRENGTH)
                    .setBrakingStart(SLOW_BRAKING_START)
                    .setGlobalDeceleration(SLOW_BRAKING_START)
                    .build();

            gateOpen2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(54.000, 84.000), new Pose(16.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .setConstraintsForLast(slow)
                    .setBrakingStrength(SLOW_BRAKING_STRENGTH)
                    .setBrakingStart(SLOW_BRAKING_START)
                    .setGlobalDeceleration(SLOW_BRAKING_START)
                    .build();

            gateMoveCollect2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(16.000, 60.000), new Pose(13.000, 50.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(120))
                    .build();

            gateExtendedCollect2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(13.000, 50.000), new Pose(13.000, 55.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(135))
                    .build();

            backToShootThird3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(13.000, 55.000), new Pose(40.000, 54.000), new Pose(54.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .setConstraintsForLast(slow)
                    .setBrakingStrength(SLOW_BRAKING_STRENGTH)
                    .setBrakingStart(SLOW_BRAKING_START)
                    .setGlobalDeceleration(SLOW_BRAKING_START)
                    .build();

            collectForth3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(54.000, 84.000), new Pose(20.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            backToShootForth3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 84.000), new Pose(54.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setConstraintsForLast(slow)
                    .setBrakingStrength(SLOW_BRAKING_STRENGTH)
                    .setBrakingStart(SLOW_BRAKING_START)
                    .setGlobalDeceleration(SLOW_BRAKING_START)
                    .build();

            path14 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(54.000, 84.000), new Pose(36.000, 80.000)))
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();
        }
    }
}