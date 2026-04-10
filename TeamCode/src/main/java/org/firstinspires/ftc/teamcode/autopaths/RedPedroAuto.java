package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autopaths.experimental.util.AutoGuards;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

@Autonomous(name = "Red 12 Ball 🔴", group = "Autonomous", preselectTeleOp = "A HORS OFFICIAL ⭐")
@Configurable
public class RedPedroAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState {
        IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, CLOSED_INTAKE_SEQUENCE,
        PRE_ACTION, INTAKE_RUN, CLAW_ACTION, WAIT_GATE_ALIGN, WAIT_GATE_CLEAR, FINISHED
    }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private Timer timedIntakeTimer;
    private Timer preActionTimer;
    private Timer poseWaitTimer;
    private Timer gateClearWaitTimer;
    private Timer gateAlignWaitTimer;

    private boolean timedIntakeActive = false;
    private boolean preActionTimerStarted = false;
    private boolean preActionEntered = false;

    private long clawActionStartMs = 0L;
    private long shooterWaitStartMs = -1;
    private long autoStartMs = -1;
    private boolean shutdownDone = false;

    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private DcMotor turretMotor;
    private DcMotor intakeMotor;
    private DcMotorEx intakeMotorEx;

    private BNO055IMU pinpointImu = null;
    private BNO055IMU hubImu = null;
    private BNO055IMU imu = null;

    private Servo clawServo;
    private Servo rightHoodServo;
    private Servo gateServo;

    private FlywheelController flywheel;
    private TurretController turretController;

    private static final double AUTO_SHOOTER_RPM = 2400;
    private boolean gateClosed = false;

    private boolean turretForceHold = true;
    private int turretHoldTarget = 0;

    // Guards
    private AutoGuards.ShooterGate shooterGate;
    private AutoGuards.JamDetector jamDetector;

    // Motion tracking
    private double lastPoseX = Double.NaN;
    private double lastPoseY = Double.NaN;
    private double lastPoseHeadingRad = Double.NaN;
    private long lastPoseTimeMs = -1L;
    private double translationalSpeedInPerS = 0.0;
    private double angularSpeedDegPerS = 0.0;

    // Timing
    @Sorter(sort = 0)  public static double INTAKE_RUN_SECONDS = 0.6;
    @Sorter(sort = 1)  public static double TIMED_INTAKE_SECONDS = 1.0;
    @Sorter(sort = 3)  public static double PRE_ACTION_WAIT_SECONDS = 1.0;
    @Sorter(sort = 4)  public static double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 1.5;
    @Sorter(sort = 5)  public static long SHOOTER_WAIT_TIMEOUT_MS = 1300L;

    // Intake power
    @Sorter(sort = 10) public static double INTAKE_ON_POWER = -0.75;
    @Sorter(sort = 11) public static double SHOOT_POSE_INTAKE_POWER = -1.0;
    @Sorter(sort = 12) public static double CLOSED_INTAKE_POWER = -0.67;
    @Sorter(sort = 13) public static double CLOSED_INTAKE_TOLERANCE_IN = 10.0;

    // Tolerances
    @Sorter(sort = 20) public static double START_POSE_TOLERANCE_IN = 2.0;

    // Gate
    @Sorter(sort = 30) public static double GATE_OPEN = 0.67;
    @Sorter(sort = 31) public static double GATE_CLOSED = 0.485;
    @Sorter(sort = 32) public static double GATE_OPEN_TOLERANCE_IN = 2.3;
    @Sorter(sort = 33) public static double GATE_CLOSE_TOLERANCE_IN = 5.0;
    @Sorter(sort = 34) public static double GATE_ALIGN_WAIT_SECONDS = 0.6;
    @Sorter(sort = 35) public static double WAIT_AFTER_GATE_CLEAR_SECONDS = 0.8;

    // Reliability
    @Sorter(sort = 36) public static double SHOOT_RPM_TOLERANCE = 50.0;
    @Sorter(sort = 37) public static double SHOOT_MAX_TRANS_SPEED = 1.5;
    @Sorter(sort = 38) public static double SHOOT_MAX_ANG_SPEED = 20.0;
    @Sorter(sort = 39) public static long SHOOT_STABLE_HOLD_MS = 150L;
    @Sorter(sort = 40) public static long FORCE_PARK_CUTOFF_MS = 26500L;

    @Sorter(sort = 41) public static double JAM_VELO_THRESHOLD = 120.0;
    @Sorter(sort = 42) public static long JAM_DETECT_MS = 160L;
    @Sorter(sort = 43) public static long JAM_CLEAR_MS = 260L;
    @Sorter(sort = 44) public static double JAM_CLEAR_POWER = 0.45;

    // Poses
    @Sorter(sort = 100) public static double START_X = 124.0;
    @Sorter(sort = 101) public static double START_Y = 122.0;
    @Sorter(sort = 102) public static double START_HEADING = 45.0;

    @Sorter(sort = 110) public static double SHOOT_POSE_X = 98.0;
    @Sorter(sort = 111) public static double SHOOT_POSE_Y = 88.0;
    @Sorter(sort = 112) public static double SHOOT_HEADING_INITIAL = 0.0;
    @Sorter(sort = 113) public static double SHOOT_HEADING_FIRST3 = 0.0;
    @Sorter(sort = 114) public static double SHOOT_SECOND3_HEADING = 0.0;
    @Sorter(sort = 115) public static double SHOOT_FINAL_HEADING = 0.0;

    @Sorter(sort = 120) public static double COLLECT_FIRST3_X = 126.0;
    @Sorter(sort = 121) public static double COLLECT_FIRST3_Y = 84.0;
    @Sorter(sort = 122) public static double COLLECT_FIRST3_HEADING = 0.0;

    @Sorter(sort = 125) public static double GATE_ALIGN_X = 126.0;
    @Sorter(sort = 126) public static double GATE_ALIGN_Y = 78.0;
    @Sorter(sort = 127) public static double GATE_ALIGN_HEADING = 0.0;

    @Sorter(sort = 130) public static double GATE_CLEAR_X = 134.0;
    @Sorter(sort = 131) public static double GATE_CLEAR_Y = 78.0;
    @Sorter(sort = 132) public static double GATE_CLEAR_HEADING = 0.0;

    @Sorter(sort = 140) public static double ALIGN_SECOND3_X = 98.0;
    @Sorter(sort = 141) public static double ALIGN_SECOND3_Y = 62.0;
    @Sorter(sort = 142) public static double ALIGN_SECOND3_HEADING = 0.0;

    @Sorter(sort = 150) public static double COLLECT_SECOND3_X = 138.0;
    @Sorter(sort = 151) public static double COLLECT_SECOND3_Y = 62.0;
    @Sorter(sort = 152) public static double COLLECT_SECOND3_HEADING = 0.0;

    @Sorter(sort = 160) public static double ALIGN_THIRD3_X = 97.0;
    @Sorter(sort = 161) public static double ALIGN_THIRD3_Y = 36.0;
    @Sorter(sort = 162) public static double ALIGN_THIRD3_HEADING = 0.0;

    @Sorter(sort = 170) public static double COLLECT_THIRD3_X = 140.0;
    @Sorter(sort = 171) public static double COLLECT_THIRD3_Y = 36.0;
    @Sorter(sort = 172) public static double COLLECT_THIRD3_HEADING = 0.0;

    @Sorter(sort = 180) public static double MOVE_RP_X = 110.0;
    @Sorter(sort = 181) public static double MOVE_RP_Y = 80.0;
    @Sorter(sort = 182) public static double MOVE_RP_HEADING = 45.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING)));

        intakeTimer = new Timer();
        timedIntakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        gateAlignWaitTimer = new Timer();
        gateClearWaitTimer = new Timer();

        shooterGate = new AutoGuards.ShooterGate(
                START_POSE_TOLERANCE_IN, SHOOT_RPM_TOLERANCE,
                SHOOT_MAX_TRANS_SPEED, SHOOT_MAX_ANG_SPEED, SHOOT_STABLE_HOLD_MS
        );
        jamDetector = new AutoGuards.JamDetector(
                JAM_VELO_THRESHOLD, JAM_DETECT_MS, JAM_CLEAR_MS, JAM_CLEAR_POWER
        );

        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
            turretMotor = hardwareMap.get(DcMotor.class, "turret");

            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            turretMotor.setDirection(DcMotor.Direction.FORWARD);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Motor map/config failed: " + e.getMessage());
        }

        try {
            try { pinpointImu = hardwareMap.get(BNO055IMU.class, "pinpoint"); } catch (Exception ignored) {}
            try { hubImu = hardwareMap.get(BNO055IMU.class, "imu"); } catch (Exception ignored) {}
            imu = (pinpointImu != null) ? pinpointImu : hubImu;

            if (imu != null) {
                BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
                imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                imu.initialize(imuParams);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "IMU init failed: " + e.getMessage());
        }

        try {
            flywheel = new FlywheelController(shooterMotor, shooterMotor2, telemetry);
            turretController = new TurretController(turretMotor, imu, telemetry);
            turretController.captureReferences();
            turretController.resetPidState();
            flywheel.setShooterOn(false);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Controller init failed: " + e.getMessage());
        }

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotorEx = (intakeMotor instanceof DcMotorEx) ? (DcMotorEx) intakeMotor : null;
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake map failed: " + e.getMessage());
        }

        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
            gateServo = hardwareMap.get(Servo.class, "gateServo");

            if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
            if (rightHoodServo != null) rightHoodServo.setPosition(0.16);
            if (gateServo != null) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Servo map failed: " + e.getMessage());
        }

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (turretController != null && turretForceHold) {
            turretController.holdPositionTicks(turretHoldTarget);
        }
    }

    @Override
    public void start() {
        autoStartMs = System.currentTimeMillis();
        shutdownDone = false;
        state = AutoState.WAIT_FOR_SHOOTER;
        shooterWaitStartMs = System.currentTimeMillis();

        if (flywheel != null) {
            flywheel.setShooterOn(true);
            flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
        }
        if (turretController != null) {
            turretController.captureReferences();
            turretController.resetPidState();
        }

        turretHoldTarget = -230;
        turretForceHold = true;

        if (shooterGate != null) shooterGate.reset();
        if (jamDetector != null) jamDetector.reset();
    }

    @Override
    public void loop() {
        follower.update();
        long nowMs = System.currentTimeMillis();

        updateChassisSpeeds(nowMs);

        if (AutoGuards.shouldForcePark(autoStartMs, nowMs, FORCE_PARK_CUTOFF_MS) && state != AutoState.FINISHED) {
            state = AutoState.FINISHED;
        }

        if (flywheel != null) {
            flywheel.handleLeftTrigger(false);
            flywheel.update(nowMs, false);
        }

        if (turretController != null && turretForceHold) {
            turretController.holdPositionTicks(turretHoldTarget);
        }

        runStateMachine(nowMs);
        updateGate();

        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
        panelsTelemetry.update(telemetry);

        if (state == AutoState.FINISHED && !shutdownDone) {
            resetToInitState();
            shutdownDone = true;
        }
    }

    @Override
    public void stop() {
        resetToInitState();
        state = AutoState.FINISHED;
    }

    private void updateChassisSpeeds(long nowMs) {
        Pose p = follower != null ? follower.getPose() : null;
        if (p == null) return;

        if (lastPoseTimeMs < 0) {
            lastPoseX = p.getX();
            lastPoseY = p.getY();
            lastPoseHeadingRad = p.getHeading();
            lastPoseTimeMs = nowMs;
            translationalSpeedInPerS = 0.0;
            angularSpeedDegPerS = 0.0;
            return;
        }

        double dt = Math.max(1e-3, (nowMs - lastPoseTimeMs) / 1000.0);
        translationalSpeedInPerS = Math.hypot(p.getX() - lastPoseX, p.getY() - lastPoseY) / dt;

        double dH = p.getHeading() - lastPoseHeadingRad;
        while (dH > Math.PI) dH -= 2.0 * Math.PI;
        while (dH < -Math.PI) dH += 2.0 * Math.PI;
        angularSpeedDegPerS = Math.abs(Math.toDegrees(dH) / dt);

        lastPoseX = p.getX();
        lastPoseY = p.getY();
        lastPoseHeadingRad = p.getHeading();
        lastPoseTimeMs = nowMs;
    }

    private boolean isReadyToShoot(long nowMs) {
        Pose current = follower != null ? follower.getPose() : null;
        Pose shoot = new Pose(SHOOT_POSE_X, SHOOT_POSE_Y, 0.0);

        double curRpm = flywheel != null ? flywheel.getCurrentRPM() : 0.0;
        double tgtRpm = flywheel != null ? flywheel.getTargetRPM() : 0.0;

        return shooterGate != null && shooterGate.isReady(
                nowMs, current, shoot, curRpm, tgtRpm, translationalSpeedInPerS, angularSpeedDegPerS
        );
    }

    private void runStateMachine(long nowMs) {
        if (timedIntakeActive && timedIntakeTimer.getElapsedTimeSeconds() >= TIMED_INTAKE_SECONDS) {
            startIntake(INTAKE_ON_POWER);
            timedIntakeActive = false;
        }

        switch (state) {
            case WAIT_FOR_SHOOTER: {
                boolean atTarget = (flywheel != null && flywheel.isAtTarget());
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (nowMs - shooterWaitStartMs);
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) startPath(1);
                break;
            }

            case RUNNING_PATH:
                if (!follower.isBusy()) {
                    int finished = currentPathIndex;

                    if (finished == 3) {
                        gateAlignWaitTimer.resetTimer();
                        nextPathIndex = 4;
                        state = AutoState.WAIT_GATE_ALIGN;
                        break;
                    }

                    if (finished == 4) {
                        gateClearWaitTimer.resetTimer();
                        nextPathIndex = 5;
                        state = AutoState.WAIT_GATE_CLEAR;
                        break;
                    }

                    if (endsAtShoot(finished)) {
                        nextPathIndex = finished + 1;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        if (shooterGate != null) shooterGate.reset();
                        state = AutoState.CLOSED_INTAKE_SEQUENCE;
                    } else {
                        int next = finished + 1;
                        if (next > 12) state = AutoState.FINISHED;
                        else startPath(next);
                    }
                }
                break;

            case WAIT_GATE_ALIGN:
                if (gateAlignWaitTimer.getElapsedTimeSeconds() >= GATE_ALIGN_WAIT_SECONDS) {
                    if (nextPathIndex > 0) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else state = AutoState.FINISHED;
                }
                break;

            case WAIT_GATE_CLEAR:
                if (gateClearWaitTimer.getElapsedTimeSeconds() >= WAIT_AFTER_GATE_CLEAR_SECONDS) {
                    if (nextPathIndex > 0) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else state = AutoState.FINISHED;
                }
                break;

            case CLOSED_INTAKE_SEQUENCE: {
                double dist = distanceToShootPose();
                if (dist <= CLOSED_INTAKE_TOLERANCE_IN) startIntake(CLOSED_INTAKE_POWER);
                if (dist <= START_POSE_TOLERANCE_IN) state = AutoState.PRE_ACTION;
                break;
            }

            case PRE_ACTION:
                if (!preActionEntered) {
                    poseWaitTimer.resetTimer();
                    preActionTimerStarted = false;
                    preActionEntered = true;
                    if (shooterGate != null) shooterGate.reset();
                }

                if (!preActionTimerStarted) {
                    boolean ready = isReadyToShoot(nowMs);
                    if (ready || poseWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_MAX_POSE_WAIT_SECONDS) {
                        preActionTimer.resetTimer();
                        preActionTimerStarted = true;
                    }
                } else if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
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
                    clawActionStartMs = nowMs;
                    state = AutoState.CLAW_ACTION;
                }
                break;

            case CLAW_ACTION:
                if (nowMs >= clawActionStartMs + ClawController.CLAW_CLOSE_MS) {
                    if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
                    if (nextPathIndex > 0 && nextPathIndex <= 12) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else state = AutoState.FINISHED;
                }
                break;

            case FINISHED:
            case IDLE:
            default:
                break;
        }
    }

    private void startPath(int idx) {
        if (idx < 1 || idx > 12) {
            currentPathIndex = 0;
            state = AutoState.FINISHED;
            return;
        }

        startIntake(INTAKE_ON_POWER);

        if (idx == 5 || idx == 8 || idx == 11) {
            timedIntakeTimer.resetTimer();
            timedIntakeActive = true;
        }

        switch (idx) {
            case 1: follower.followPath(paths.startToShoot); break;
            case 2: follower.followPath(paths.collectFirst3); break;
            case 3: follower.followPath(paths.gateAlign); break;
            case 4: follower.followPath(paths.gateClear); break;
            case 5: follower.followPath(paths.backToShootFirst3); break;
            case 6: follower.followPath(paths.alignToCollectSecond3); break;
            case 7: follower.followPath(paths.collectSecond3); break;
            case 8: follower.followPath(paths.backToShootSecond3); break;
            case 9: follower.followPath(paths.alignToCollectThird3); break;
            case 10: follower.followPath(paths.collectThird3); break;
            case 11: follower.followPath(paths.backToShootThird3); break;
            case 12: follower.followPath(paths.moveForRP); break;
            default: break;
        }

        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
    }

    private boolean endsAtShoot(int pathIndex) {
        return pathIndex == 1 || pathIndex == 5 || pathIndex == 8 || pathIndex == 11;
    }

    private double distanceToShootPose() {
        try {
            Pose p = follower.getPose();
            return Math.hypot(p.getX() - SHOOT_POSE_X, p.getY() - SHOOT_POSE_Y);
        } catch (Exception e) {
            return Double.POSITIVE_INFINITY;
        }
    }

    private void startIntake(double power) {
        try {
            double finalPower = power;
            if (jamDetector != null) {
                double measuredVelocity = 0.0;
                try { if (intakeMotorEx != null) measuredVelocity = intakeMotorEx.getVelocity(); }
                catch (Exception ignored) {}
                finalPower = jamDetector.update(System.currentTimeMillis(), power, measuredVelocity);
            }
            if (intakeMotor != null) intakeMotor.setPower(finalPower);
        } catch (Exception e) {
            panelsTelemetry.debug("Intake", "startIntake error: " + e.getMessage());
        }
    }

    private void stopIntake() {
        try {
            if (intakeMotor != null) intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Intake", "stopIntake error: " + e.getMessage());
        }
    }

    private void updateGate() {
        try {
            double dist = distanceToShootPose();
            if (dist <= GATE_OPEN_TOLERANCE_IN && gateServo != null && gateClosed) {
                gateServo.setPosition(GATE_OPEN);
                gateClosed = false;
            } else if (dist >= GATE_CLOSE_TOLERANCE_IN && gateServo != null && !gateClosed) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Gate", "updateGate error: " + e.getMessage());
        }
    }

    private void resetToInitState() {
        if (flywheel != null) {
            flywheel.setShooterOn(false);
            flywheel.setTargetRPM(0.0);
            flywheel.update(System.currentTimeMillis(), false);
        }
        stopIntake();

        if (gateServo != null) {
            gateServo.setPosition(GATE_CLOSED);
            gateClosed = true;
        }
        if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
        if (rightHoodServo != null) rightHoodServo.setPosition(0.16);

        if (turretController != null && turretForceHold) {
            turretHoldTarget = 0;
            turretController.holdPositionTicks(turretHoldTarget);
        }

        if (turretMotor != null) {
            try { turretMotor.setPower(0.0); } catch (Exception ignored) {}
        }
    }

    public static class Paths {
        public PathChain startToShoot;
        public PathChain collectFirst3;
        public PathChain gateAlign;
        public PathChain gateClear;
        public PathChain backToShootFirst3;
        public PathChain alignToCollectSecond3;
        public PathChain collectSecond3;
        public PathChain backToShootSecond3;
        public PathChain alignToCollectThird3;
        public PathChain collectThird3;
        public PathChain backToShootThird3;
        public PathChain moveForRP;

        public Paths(Follower follower) {
            startToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(START_X, START_Y), new Pose(SHOOT_POSE_X, SHOOT_POSE_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(START_HEADING), Math.toRadians(SHOOT_HEADING_INITIAL))
                    .build();

            collectFirst3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(SHOOT_POSE_X, SHOOT_POSE_Y), new Pose(COLLECT_FIRST3_X, COLLECT_FIRST3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_INITIAL), Math.toRadians(COLLECT_FIRST3_HEADING))
                    .build();

            gateAlign = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(COLLECT_FIRST3_X, COLLECT_FIRST3_Y), new Pose(GATE_ALIGN_X, GATE_ALIGN_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_FIRST3_HEADING), Math.toRadians(GATE_ALIGN_HEADING))
                    .setBrakingStart(.55)
                    .setBrakingStrength(1.0)
                    .build();

            gateClear = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(GATE_ALIGN_X, GATE_ALIGN_Y), new Pose(GATE_CLEAR_X, GATE_CLEAR_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(GATE_ALIGN_HEADING), Math.toRadians(GATE_CLEAR_HEADING))
                    .build();

            backToShootFirst3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(GATE_CLEAR_X, GATE_CLEAR_Y), new Pose(SHOOT_POSE_X, SHOOT_POSE_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(GATE_CLEAR_HEADING), Math.toRadians(SHOOT_HEADING_FIRST3))
                    .build();

            alignToCollectSecond3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(SHOOT_POSE_X, SHOOT_POSE_Y), new Pose(ALIGN_SECOND3_X, ALIGN_SECOND3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_FIRST3), Math.toRadians(ALIGN_SECOND3_HEADING))
                    .build();

            collectSecond3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(ALIGN_SECOND3_X, ALIGN_SECOND3_Y), new Pose(COLLECT_SECOND3_X, COLLECT_SECOND3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(ALIGN_SECOND3_HEADING), Math.toRadians(COLLECT_SECOND3_HEADING))
                    .build();

            backToShootSecond3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(COLLECT_SECOND3_X, COLLECT_SECOND3_Y), new Pose(SHOOT_POSE_X, SHOOT_POSE_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_SECOND3_HEADING), Math.toRadians(SHOOT_SECOND3_HEADING))
                    .build();//re write logic for this path

            alignToCollectThird3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(SHOOT_POSE_X, SHOOT_POSE_Y), new Pose(ALIGN_THIRD3_X, ALIGN_THIRD3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_SECOND3_HEADING), Math.toRadians(ALIGN_THIRD3_HEADING))
                    .build();

            collectThird3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(ALIGN_THIRD3_X, ALIGN_THIRD3_Y), new Pose(COLLECT_THIRD3_X, COLLECT_THIRD3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(ALIGN_THIRD3_HEADING), Math.toRadians(COLLECT_THIRD3_HEADING))
                    .build();

            backToShootThird3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(COLLECT_THIRD3_X, COLLECT_THIRD3_Y), new Pose(SHOOT_POSE_X, SHOOT_POSE_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_THIRD3_HEADING), Math.toRadians(SHOOT_FINAL_HEADING))
                    .build();

            moveForRP = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(SHOOT_POSE_X, SHOOT_POSE_Y), new Pose(MOVE_RP_X, MOVE_RP_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_FINAL_HEADING), Math.toRadians(MOVE_RP_HEADING))
                    .build();
        }
    }
}