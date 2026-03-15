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
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

@Autonomous(name = "Far 12 🔵", group = "Autonomous", preselectTeleOp = "HORS OFFICIAL ⭐")
@Configurable
public class FarBlue12 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, PRE_ACTION, INTAKE_RUN, CLAW_ACTION, WAIT_AFTER_SHOOT, FINISHED }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private Timer preActionTimer;
    private Timer poseWaitTimer;
    private Timer rpmStableTimer;
    private Timer waitAfterShootTimer;

    private boolean preActionTimerStarted = false;
    private boolean preActionEntered = false;

    private long clawActionStartMs = 0L;
    private long shooterWaitStartMs = -1;

    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private DcMotor turretMotor;
    private FlywheelController flywheel;
    private static final double AUTO_SHOOTER_RPM = FlywheelController.TARGET_RPM_FAR;

    private BNO055IMU imu = null;
    private BNO055IMU pinpointImu = null;
    private BNO055IMU hubImu = null;

    private TurretController turretController;
    private static final int TURRET_START_POS = -107;
    private boolean turretRefCaptured = false;

    private DcMotor intakeMotor;
    private Servo clawServo;
    private Servo gateServo;
    private Servo rightHoodServo;
    private boolean gateClosed = true;

    private long autoStartMs = -1;
    private int lastFinishedShootPath = -1;

    // ============================
    // Timing
    // ============================
    @Sorter(sort = 0)  public static double INTAKE_RUN_SECONDS = 1.6;
    @Sorter(sort = 2)  public static double PRE_ACTION_WAIT_SECONDS = 1.0;
    @Sorter(sort = 3)  public static double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 1.8;
    @Sorter(sort = 4)  public static long   SHOOTER_WAIT_TIMEOUT_MS = 1400L;

    // ============================
    // Wait after each shoot sequence
    // ============================
    @Sorter(sort = 5)  public static double WAIT_AFTER_FIRST_SECONDS  = 2.0;
    @Sorter(sort = 6)  public static double WAIT_AFTER_SECOND_SECONDS = 2.0;
    @Sorter(sort = 7)  public static double WAIT_AFTER_THIRD_SECONDS  = 2.0;

    // ============================
    // Intake power rules
    // ============================
    @Sorter(sort = 10) public static double INTAKE_IDLE_POWER    = -0.50;
    @Sorter(sort = 11) public static double INTAKE_SHOOT_POWER   = -0.55;
    @Sorter(sort = 12) public static double INTAKE_COLLECT_POWER = -1.00;

    // ============================
    // Gate settings
    // ============================
    @Sorter(sort = 20) public static double GATE_OPEN = 0.67;
    @Sorter(sort = 21) public static double GATE_CLOSED = 0.50;
    @Sorter(sort = 22) public static double GATE_OPEN_TOLERANCE_IN = 3.0;
    @Sorter(sort = 23) public static double GATE_CLOSE_TOLERANCE_IN = 6.0;
    @Sorter(sort = 24) public static double GATE_RPM_TOLERANCE = 40.0;
    @Sorter(sort = 25) public static double GATE_RPM_STABLE_SECONDS = 0.35;

    // ============================
    // Pose tolerances
    // ============================
    @Sorter(sort = 30) public static double START_POSE_TOLERANCE_IN = 6.0;

    // ========================================
    // PATH POSES - START
    // ========================================
    @Sorter(sort = 100) public static double START_X = 63.000;
    @Sorter(sort = 101) public static double START_Y = 8.000;
    @Sorter(sort = 102) public static double START_HEADING_DEG = 90.0;

    // ========================================
    // PATH POSES - SHOOT
    // ========================================
    @Sorter(sort = 110) public static double SHOOT_X = 58.000;
    @Sorter(sort = 111) public static double SHOOT_Y = 14.000;
    @Sorter(sort = 112) public static double SHOOT_HEADING_DEG = 90.0;

    // ========================================
    // PATH POSES - COLLECT FIRST 3
    // ========================================
    @Sorter(sort = 120) public static double COLLECT_FIRST3_X = 14.000;
    @Sorter(sort = 121) public static double COLLECT_FIRST3_Y = 22.000;
    @Sorter(sort = 122) public static double COLLECT_FIRST3_HEADING_DEG = 245.0;

    // Bezier control: Shoot -> Collect First 3
    @Sorter(sort = 123) public static double SHOOT_TO_COLLECT_FIRST3_CTRL_X = 49.367;
    @Sorter(sort = 124) public static double SHOOT_TO_COLLECT_FIRST3_CTRL_Y = 7.423;

    // ========================================
    // PATH POSES - EXTEND FIRST 3 COLLECT
    // ========================================
    @Sorter(sort = 130) public static double EXTEND_FIRST3_X = 13.163;
    @Sorter(sort = 131) public static double EXTEND_FIRST3_Y = 11.833;
    @Sorter(sort = 132) public static double EXTEND_FIRST3_HEADING_DEG = 270.0;

    // Bezier control: Collect First 3 -> Extend First 3
    @Sorter(sort = 133) public static double COLLECT_FIRST3_TO_EXTEND_CTRL_X = 10.000;
    @Sorter(sort = 134) public static double COLLECT_FIRST3_TO_EXTEND_CTRL_Y = 16.000;

    // ========================================
    // PATH POSES - COLLECT SECOND 3
    // ========================================
    @Sorter(sort = 140) public static double COLLECT_SECOND3_X = 10.865;
    @Sorter(sort = 141) public static double COLLECT_SECOND3_Y = 21.577;
    @Sorter(sort = 142) public static double COLLECT_SECOND3_HEADING_DEG = 180.0;

    // Bezier control: Shoot -> Collect Second 3
    @Sorter(sort = 143) public static double SHOOT_TO_COLLECT_SECOND3_CTRL_X = 56.042;
    @Sorter(sort = 144) public static double SHOOT_TO_COLLECT_SECOND3_CTRL_Y = 22.067;

    // ========================================
    // PATH POSES - COLLECT THIRD 3 (reuses second3 geometry)
    // ========================================
    @Sorter(sort = 150) public static double COLLECT_THIRD3_X = COLLECT_SECOND3_X;
    @Sorter(sort = 151) public static double COLLECT_THIRD3_Y = COLLECT_SECOND3_Y;
    @Sorter(sort = 152) public static double COLLECT_THIRD3_HEADING_DEG = COLLECT_SECOND3_HEADING_DEG;

    // ========================================
    // PATH POSES - MOVE FOR RP
    // ========================================
    @Sorter(sort = 160) public static double MOVE_RP_X = 31.684;
    @Sorter(sort = 161) public static double MOVE_RP_Y = 14.730;
    @Sorter(sort = 162) public static double MOVE_RP_HEADING_DEG = 90.0;

    public FarBlue12() {}

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        intakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        rpmStableTimer = new Timer();
        waitAfterShootTimer = new Timer();

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
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Failed to map shooter/turret motors: " + e.getMessage());
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
            if (shooterMotor != null) flywheel = new FlywheelController(shooterMotor, shooterMotor2, telemetry);
            if (turretMotor != null) turretController = new TurretController(turretMotor, imu, telemetry);
            if (turretController != null) {
                turretController.captureReferences();
                turretController.resetPidState();
            }
            if (flywheel != null) flywheel.setShooterOn(false);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Flywheel/TurretController creation error: " + e.getMessage());
        }

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake map failed: " + e.getMessage());
        }

        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Claw servo map failed: " + e.getMessage());
        }

        try {
            rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
            if (rightHoodServo != null) rightHoodServo.setPosition(0.16);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Hood servo map failed: " + e.getMessage());
        }

        try {
            gateServo = hardwareMap.get(Servo.class, "gateServo");
            if (gateServo != null) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Gate servo map failed: " + e.getMessage());
        }

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (turretController != null) turretController.update(false, 0.0);
    }

    @Override
    public void start() {
        autoStartMs = System.currentTimeMillis();
        if (flywheel != null) {
            flywheel.setShooterOn(true);
            flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
        }
        if (turretController != null) {
            turretController.captureReferences();
            turretController.resetPidState();
        }
        shooterWaitStartMs = System.currentTimeMillis();
        state = AutoState.WAIT_FOR_SHOOTER;
    }

    @Override
    public void loop() {
        follower.update();
        long nowMs = System.currentTimeMillis();

        if (turretController != null && !turretRefCaptured) {
            boolean done = turretController.driveToPosition(TURRET_START_POS, 10, 0.35);
            if (done) {
                turretController.captureReferences();
                turretRefCaptured = true;
            }
        } else if (turretController != null) {
            turretController.update(false, 0.0);
        }

        if (flywheel != null) {
            flywheel.handleLeftTrigger(false);
            flywheel.update(nowMs, false);
        }

        runStateMachine(nowMs);
        updateGate();
        applyIntakePolicy();

        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        resetToInitState();
        state = AutoState.FINISHED;
    }

    private void resetToInitState() {
        if (flywheel != null) {
            flywheel.setShooterOn(false);
            flywheel.setTargetRPM(0.0);
            flywheel.update(System.currentTimeMillis(), false);
        }
        stopIntake();
        if (gateServo != null) { gateServo.setPosition(GATE_CLOSED); gateClosed = true; }
        if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
        if (rightHoodServo != null) rightHoodServo.setPosition(0.16);
        if (turretMotor != null) { try { turretMotor.setPower(0.0); } catch (Exception ignored) {} }
    }

    private void runStateMachine(long nowMs) {
        switch (state) {
            case WAIT_FOR_SHOOTER: {
                boolean atTarget = flywheel != null && flywheel.isAtTarget();
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (System.currentTimeMillis() - shooterWaitStartMs);
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) startPath(1);
                break;
            }

            case RUNNING_PATH: {
                if (!follower.isBusy()) {
                    int finished = currentPathIndex;
                    int next = finished + 1;

                    if (endsAtShoot(finished)) {
                        nextPathIndex = (next <= 9) ? next : -1;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        poseWaitTimer.resetTimer();
                        rpmStableTimer.resetTimer();
                        if (flywheel != null) flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
                        state = AutoState.PRE_ACTION;
                    } else if (finished == 9) {
                        state = AutoState.FINISHED;
                    } else if (next <= 9) {
                        startPath(next);
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;
            }

            case PRE_ACTION: {
                if (!preActionEntered) {
                    poseWaitTimer.resetTimer();
                    preActionTimerStarted = false;
                    preActionEntered = true;
                }

                if (!preActionTimerStarted) {
                    double dist = distanceToShootPose();
                    if (dist <= START_POSE_TOLERANCE_IN || poseWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_MAX_POSE_WAIT_SECONDS) {
                        preActionTimer.resetTimer();
                        preActionTimerStarted = true;
                    }
                } else {
                    if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                        intakeTimer.resetTimer();
                        state = AutoState.INTAKE_RUN;
                    }
                }
                break;
            }

            case INTAKE_RUN: {
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                    if (clawServo != null) clawServo.setPosition(ClawController.CLAW_CLOSED);
                    clawActionStartMs = System.currentTimeMillis();
                    state = AutoState.CLAW_ACTION;
                }
                break;
            }

            case CLAW_ACTION: {
                if (System.currentTimeMillis() >= clawActionStartMs + ClawController.CLAW_CLOSE_MS) {
                    if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
                    lastFinishedShootPath = currentPathIndex;
                    waitAfterShootTimer.resetTimer();
                    state = AutoState.WAIT_AFTER_SHOOT;
                }
                break;
            }

            case WAIT_AFTER_SHOOT: {
                double waitSeconds = getWaitAfterShootSeconds(lastFinishedShootPath);
                if (waitAfterShootTimer.getElapsedTimeSeconds() >= waitSeconds) {
                    if (nextPathIndex > 0 && nextPathIndex <= 9) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;
            }

            case FINISHED:
            case IDLE:
            default:
                break;
        }
    }

    private double getWaitAfterShootSeconds(int shootPathIdx) {
        switch (shootPathIdx) {
            case 1: return WAIT_AFTER_FIRST_SECONDS;
            case 4: return WAIT_AFTER_SECOND_SECONDS;
            case 6: return WAIT_AFTER_THIRD_SECONDS;
            case 8: return 0.0;
            default: return 0.0;
        }
    }

    private void startPath(int idx) {
        switch (idx) {
            case 1: follower.followPath(paths.startToShoot);          break;
            case 2: follower.followPath(paths.shootToCollectFirst3);  break;
            case 3: follower.followPath(paths.extendCollectFirst3);   break;
            case 4: follower.followPath(paths.backShootFirst3);       break;
            case 5: follower.followPath(paths.collectSecond3);        break;
            case 6: follower.followPath(paths.backShootSecond3);      break;
            case 7: follower.followPath(paths.collectThird3);         break;
            case 8: follower.followPath(paths.backShootThird3);       break;
            case 9: follower.followPath(paths.moveForRP);             break;
            default: state = AutoState.FINISHED; return;
        }
        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
    }

    private boolean endsAtShoot(int pathIdx) {
        return pathIdx == 1 || pathIdx == 4 || pathIdx == 6 || pathIdx == 8;
    }

    private boolean isCollectPath(int pathIdx) {
        return pathIdx == 2 || pathIdx == 3 || pathIdx == 5 || pathIdx == 7;
    }

    private void applyIntakePolicy() {
        double desired;
        boolean inShootPhase = (state == AutoState.PRE_ACTION || state == AutoState.INTAKE_RUN
                || state == AutoState.CLAW_ACTION || state == AutoState.WAIT_AFTER_SHOOT
                || endsAtShoot(currentPathIndex));
        boolean inCollectPath = isCollectPath(currentPathIndex);

        if (inCollectPath) {
            desired = INTAKE_COLLECT_POWER;
        } else if (inShootPhase) {
            desired = INTAKE_SHOOT_POWER;
        } else {
            desired = INTAKE_IDLE_POWER;
        }
        startIntake(desired);
    }

    private void startIntake(double power) {
        try {
            if (intakeMotor != null) intakeMotor.setPower(power);
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

    private double distanceToShootPose() {
        try {
            Pose p = follower.getPose();
            double dx = p.getX() - SHOOT_X;
            double dy = p.getY() - SHOOT_Y;
            return Math.hypot(dx, dy);
        } catch (Exception e) {
            return Double.POSITIVE_INFINITY;
        }
    }

    private void updateGate() {
        try {
            double dist = distanceToShootPose();

            boolean inShootPhase = (state == AutoState.PRE_ACTION
                    || state == AutoState.INTAKE_RUN
                    || state == AutoState.CLAW_ACTION);

            boolean rpmCloseEnough = false;
            if (flywheel != null && flywheel.isShooterOn()) {
                double err = Math.abs(flywheel.getCurrentRPM() - flywheel.getTargetRPM());
                if (err <= GATE_RPM_TOLERANCE) {
                    rpmCloseEnough = true;
                } else if (!inShootPhase) {
                    rpmStableTimer.resetTimer();
                }
            } else {
                rpmStableTimer.resetTimer();
            }

            boolean rpmStable = rpmCloseEnough
                    && rpmStableTimer.getElapsedTimeSeconds() >= GATE_RPM_STABLE_SECONDS;

            boolean shouldBeOpen = inShootPhase
                    && dist <= GATE_OPEN_TOLERANCE_IN
                    && rpmStable;

            boolean shouldBeClosed = !inShootPhase
                    || dist >= GATE_CLOSE_TOLERANCE_IN;

            if (shouldBeOpen && gateClosed && gateServo != null) {
                gateServo.setPosition(GATE_OPEN);
                gateClosed = false;
            } else if (shouldBeClosed && !gateClosed && gateServo != null) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
            }

        } catch (Exception e) {
            panelsTelemetry.debug("Gate", "updateGate error: " + e.getMessage());
        }
    }

    public static class Paths {
        public PathChain startToShoot;
        public PathChain shootToCollectFirst3;
        public PathChain extendCollectFirst3;
        public PathChain backShootFirst3;
        public PathChain collectSecond3;
        public PathChain backShootSecond3;
        public PathChain collectThird3;
        public PathChain backShootThird3;
        public PathChain moveForRP;

        public Paths(Follower follower) {
            startToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(START_X, START_Y),
                            new Pose(SHOOT_X, SHOOT_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(START_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                    .build();

            shootToCollectFirst3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(SHOOT_X, SHOOT_Y),
                            new Pose(SHOOT_TO_COLLECT_FIRST3_CTRL_X, SHOOT_TO_COLLECT_FIRST3_CTRL_Y),
                            new Pose(COLLECT_FIRST3_X, COLLECT_FIRST3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(COLLECT_FIRST3_HEADING_DEG))
                    .build();

            extendCollectFirst3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(COLLECT_FIRST3_X, COLLECT_FIRST3_Y),
                            new Pose(COLLECT_FIRST3_TO_EXTEND_CTRL_X, COLLECT_FIRST3_TO_EXTEND_CTRL_Y),
                            new Pose(EXTEND_FIRST3_X, EXTEND_FIRST3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_FIRST3_HEADING_DEG), Math.toRadians(EXTEND_FIRST3_HEADING_DEG))
                    .build();

            backShootFirst3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(EXTEND_FIRST3_X, EXTEND_FIRST3_Y),
                            new Pose(SHOOT_X, SHOOT_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(EXTEND_FIRST3_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                    .build();

            collectSecond3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(SHOOT_X, SHOOT_Y),
                            new Pose(SHOOT_TO_COLLECT_SECOND3_CTRL_X, SHOOT_TO_COLLECT_SECOND3_CTRL_Y),
                            new Pose(COLLECT_SECOND3_X, COLLECT_SECOND3_Y)))
                    .setConstantHeadingInterpolation(Math.toRadians(COLLECT_SECOND3_HEADING_DEG))
                    .build();

            backShootSecond3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(COLLECT_SECOND3_X, COLLECT_SECOND3_Y),
                            new Pose(SHOOT_X, SHOOT_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_SECOND3_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                    .build();

            // same geometry as current behavior
            collectThird3 = collectSecond3;
            backShootThird3 = backShootSecond3;

            moveForRP = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(SHOOT_X, SHOOT_Y),
                            new Pose(MOVE_RP_X, MOVE_RP_Y)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}