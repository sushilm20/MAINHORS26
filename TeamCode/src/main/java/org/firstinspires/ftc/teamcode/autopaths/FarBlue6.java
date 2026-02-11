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
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

@Autonomous(name = "Far 9?? 🔷", group = "Autonomous", preselectTeleOp ="HORS OFFICIAL ⭐")
@Configurable
public class FarBlue6 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, PRE_ACTION, INTAKE_RUN, CLAW_ACTION, FINISHED }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private Timer preActionTimer;
    private Timer poseWaitTimer;
    private Timer rpmStableTimer;

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
    private static final int TURRET_START_POS = -86;
    private boolean turretRefCaptured = false;

    private DcMotor intakeMotor;
    private Servo clawServo;
    private Servo gateServo;
    private Servo rightHoodServo;
    private boolean gateClosed = true;

    private long autoStartMs = -1;

    // ============================
    // Timing
    // ============================
    @Sorter(sort = 0)  public static double INTAKE_RUN_SECONDS = 1.6; // was 0.6, +1s
    @Sorter(sort = 1)  public static long   CLAW_CLOSE_MS = 190L;
    @Sorter(sort = 2)  public static double PRE_ACTION_WAIT_SECONDS = 1.1;
    @Sorter(sort = 3)  public static double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 1.8;
    @Sorter(sort = 4)  public static long   SHOOTER_WAIT_TIMEOUT_MS = 1400L;

    // ============================
    // Intake power rules
    // ============================
    @Sorter(sort = 10) public static double INTAKE_IDLE_POWER   = -0.50;
    @Sorter(sort = 11) public static double INTAKE_SHOOT_POWER  = -0.55; // slowed feed
    @Sorter(sort = 12) public static double INTAKE_COLLECT_POWER= -1.00;

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

    // ============================
    // Path poses (fixed to your sequence)
    // ============================
    @Sorter(sort = 100) public static double START_X = 63.0;
    @Sorter(sort = 101) public static double START_Y = 8.0;
    @Sorter(sort = 102) public static double START_HEADING_DEG = 90.0;

    @Sorter(sort = 110) public static double SHOOT_X = 58.0;
    @Sorter(sort = 111) public static double SHOOT_Y = 14.0;
    @Sorter(sort = 112) public static double SHOOT_HEADING_DEG = 90.0;
    @Sorter(sort = 113) public static double SHOOT_RETURN_HEADING_DEG = 180.0;

    @Sorter(sort = 120) public static double COLLECT_CTRL_X = 27.0;
    @Sorter(sort = 121) public static double COLLECT_CTRL_Y = 18.0;
    @Sorter(sort = 122) public static double COLLECT_END_X = 16.0;
    @Sorter(sort = 123) public static double COLLECT_END_Y = 14.0;
    @Sorter(sort = 124) public static double COLLECT_HEADING_DEG = 195.0;

    @Sorter(sort = 130) public static double EXTEND_END_Y = 10.392;

    @Sorter(sort = 140) public static double BACK_COLLECT_X = 16.0;
    @Sorter(sort = 141) public static double BACK_COLLECT_Y = 14.0;

    public FarBlue6() {}

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
            try {
                pinpointImu = hardwareMap.get(BNO055IMU.class, "pinpoint");
                panelsTelemetry.debug("Init", "Found PinPoint IMU as 'pinpoint'");
            } catch (Exception e) {
                panelsTelemetry.debug("Init", "PinPoint IMU not found: " + e.getMessage());
            }
            try {
                hubImu = hardwareMap.get(BNO055IMU.class, "imu");
                panelsTelemetry.debug("Init", "Found expansion hub IMU as 'imu'");
            } catch (Exception e) {
                panelsTelemetry.debug("Init", "Expansion hub IMU not found: " + e.getMessage());
            }
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
            if (clawServo != null) clawServo.setPosition(0.63);
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

        // Turret pre-position then capture reference heading
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
        if (flywheel != null) {
            panelsTelemetry.debug("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
            panelsTelemetry.debug("Fly Target", String.format("%.1f", flywheel.getTargetRPM()));
        }
        if (turretController != null) {
            panelsTelemetry.debug("Turret Enc", turretMotor.getCurrentPosition());
            panelsTelemetry.debug("TurretRefCaptured", String.valueOf(turretRefCaptured));
        }
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
        if (clawServo != null) clawServo.setPosition(0.63);
        if (rightHoodServo != null) rightHoodServo.setPosition(0.16);
        if (turretMotor != null) { try { turretMotor.setPower(0.0); } catch (Exception ignored) {} }
    }

    // ---------- State Machine ----------
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
                        nextPathIndex = (next <= 6) ? next : -1;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        poseWaitTimer.resetTimer();
                        rpmStableTimer.resetTimer();
                        if (flywheel != null) flywheel.setTargetRPM(AUTO_SHOOTER_RPM); // ensure full spin before volley
                        state = AutoState.PRE_ACTION;
                    } else if (next <= 6) {
                        startPath(next);
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;
            }

            case PRE_ACTION: {
                // Enter once per volley
                if (!preActionEntered) {
                    poseWaitTimer.resetTimer();
                    preActionTimerStarted = false;
                    preActionEntered = true;
                }

                // Start the short settle timer once close enough OR after max wait
                if (!preActionTimerStarted) {
                    double dist = distanceToShootPose();
                    if (dist <= START_POSE_TOLERANCE_IN || poseWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_MAX_POSE_WAIT_SECONDS) {
                        preActionTimer.resetTimer();
                        preActionTimerStarted = true;
                    }
                } else {
                    // Watchdog: never hang here
                    if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                        intakeTimer.resetTimer();
                        state = AutoState.INTAKE_RUN;
                    }
                }
                break;
            }

            case INTAKE_RUN: {
                // Watchdog: proceed after timer
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                    if (flywheel != null) flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM); // slight drop while feeding
                    if (clawServo != null) clawServo.setPosition(0.2);
                    clawActionStartMs = System.currentTimeMillis();
                    state = AutoState.CLAW_ACTION;
                }
                break;
            }

            case CLAW_ACTION: {
                if (System.currentTimeMillis() >= clawActionStartMs + CLAW_CLOSE_MS) {
                    if (clawServo != null) clawServo.setPosition(0.63);
                    if (nextPathIndex > 0 && nextPathIndex <= 6) {
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

    private void startPath(int idx) {
        switch (idx) {
            case 1: follower.followPath(paths.StartTOShoot3); break;
            case 2: follower.followPath(paths.Collect3); break;
            case 3: follower.followPath(paths.ExtendCollect3); break;
            case 4: follower.followPath(paths.BackTOShoot); break;
            case 5: follower.followPath(paths.BackTOCollect); break;
            case 6: follower.followPath(paths.BackTOShootFinal); break;
            default: state = AutoState.FINISHED; return;
        }
        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
    }

    private boolean endsAtShoot(int pathIdx) {
        return pathIdx == 1 || pathIdx == 4 || pathIdx == 6;
    }

    // ---------- Intake policy ----------
    private void applyIntakePolicy() {
        double desired = INTAKE_IDLE_POWER;
        boolean inShootPhase = (state == AutoState.PRE_ACTION || state == AutoState.INTAKE_RUN || state == AutoState.CLAW_ACTION || endsAtShoot(currentPathIndex));
        boolean inCollectPath = currentPathIndex == 2 || currentPathIndex == 3 || currentPathIndex == 5;

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

    // ---------- Gate control with RPM stability ----------
    private void updateGate() {
        try {
            double dist = distanceToShootPose();
            boolean rpmStable = false;
            if (flywheel != null && flywheel.isShooterOn()) {
                double err = Math.abs(flywheel.getCurrentRPM() - flywheel.getTargetRPM());
                if (err <= GATE_RPM_TOLERANCE) {
                    if (rpmStableTimer.getElapsedTimeSeconds() >= GATE_RPM_STABLE_SECONDS) {
                        rpmStable = true;
                    }
                } else {
                    rpmStableTimer.resetTimer();
                }
            } else {
                rpmStableTimer.resetTimer();
            }

            if (dist <= GATE_OPEN_TOLERANCE_IN && rpmStable && gateServo != null && gateClosed) {
                gateServo.setPosition(GATE_OPEN);
                gateClosed = false;
                panelsTelemetry.debug("Gate", "Opened (dist=" + String.format("%.2f", dist) + ", rpmStable=" + rpmStable + ")");
            } else if ((dist >= GATE_CLOSE_TOLERANCE_IN || !rpmStable) && gateServo != null && !gateClosed) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
                panelsTelemetry.debug("Gate", "Closed (dist=" + String.format("%.2f", dist) + ", rpmStable=" + rpmStable + ")");
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Gate", "updateGate error: " + e.getMessage());
        }
    }

    // ---------- Paths ----------
    public static class Paths {
        public PathChain StartTOShoot3;
        public PathChain Collect3;
        public PathChain ExtendCollect3;
        public PathChain BackTOShoot;
        public PathChain BackTOCollect;
        public PathChain BackTOShootFinal;

        public Paths(Follower follower) {
            StartTOShoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(START_X, START_Y),
                            new Pose(SHOOT_X, SHOOT_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(START_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                    .build();

            Collect3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(SHOOT_X, SHOOT_Y),
                            new Pose(COLLECT_CTRL_X, COLLECT_CTRL_Y),
                            new Pose(COLLECT_END_X, COLLECT_END_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(COLLECT_HEADING_DEG))
                    .build();

            ExtendCollect3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(COLLECT_END_X, COLLECT_END_Y),
                            new Pose(COLLECT_END_X, EXTEND_END_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_HEADING_DEG), Math.toRadians(COLLECT_HEADING_DEG))
                    .build();

            BackTOShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(COLLECT_END_X, EXTEND_END_Y),
                            new Pose(SHOOT_X, SHOOT_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_HEADING_DEG), Math.toRadians(SHOOT_RETURN_HEADING_DEG))
                    .build();

            BackTOCollect = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(SHOOT_X, SHOOT_Y),
                            new Pose(BACK_COLLECT_X, BACK_COLLECT_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_RETURN_HEADING_DEG), Math.toRadians(COLLECT_HEADING_DEG))
                    .build();

            BackTOShootFinal = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(BACK_COLLECT_X, BACK_COLLECT_Y),
                            new Pose(SHOOT_X, SHOOT_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_HEADING_DEG), Math.toRadians(SHOOT_RETURN_HEADING_DEG))
                    .build();
        }
    }
}