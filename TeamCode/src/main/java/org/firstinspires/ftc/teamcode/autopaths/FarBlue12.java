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

    // Track which shoot volley just finished (for wait-after-shoot)
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
    @Sorter(sort = 10) public static double INTAKE_IDLE_POWER   = -0.50;
    @Sorter(sort = 11) public static double INTAKE_SHOOT_POWER  = -0.55;
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

    // ============================
    // Path poses
    // ============================
    @Sorter(sort = 100) public static double START_X = 63.0;
    @Sorter(sort = 101) public static double START_Y = 8.0;
    @Sorter(sort = 102) public static double START_HEADING_DEG = 90.0;

    @Sorter(sort = 110) public static double SHOOT_X = 58.0;
    @Sorter(sort = 111) public static double SHOOT_Y = 14.0;
    @Sorter(sort = 112) public static double SHOOT_HEADING_DEG = 90.0;

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
        if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
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
                        // Determine the next path after the shoot+wait
                        nextPathIndex = (next <= 9) ? next : -1;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        poseWaitTimer.resetTimer();
                        rpmStableTimer.resetTimer();
                        if (flywheel != null) flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
                        state = AutoState.PRE_ACTION;
                    } else if (finished == 9) {
                        // MoveForRP just finished
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
                    if (clawServo != null) clawServo.setPosition(ClawController.CLAW_CLOSED);
                    clawActionStartMs = System.currentTimeMillis();
                    state = AutoState.CLAW_ACTION;
                }
                break;
            }

            case CLAW_ACTION: {
                if (System.currentTimeMillis() >= clawActionStartMs + ClawController.CLAW_CLOSE_MS) {
                    if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
                    // Remember which shoot path just finished so we pick the right wait duration
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

    /**
     * Returns the configurable wait duration (seconds) after the shoot
     * sequence that ended on the given path index.
     *
     * Path 1  = StartToShoot         → WAIT_AFTER_FIRST
     * Path 4  = BackShootFirst3      → WAIT_AFTER_SECOND
     * Path 6  = BackShootSecond3     → WAIT_AFTER_THIRD
     * Path 8  = BackShootThird3      → no more collect, go straight to MoveForRP (0 wait)
     */
    private double getWaitAfterShootSeconds(int shootPathIdx) {
        switch (shootPathIdx) {
            case 1: return WAIT_AFTER_FIRST_SECONDS;
            case 4: return WAIT_AFTER_SECOND_SECONDS;
            case 6: return WAIT_AFTER_THIRD_SECONDS;
            case 8: return 0.0; // last shoot → MoveForRP immediately
            default: return 0.0;
        }
    }

    /**
     * Path index mapping:
     *  1 = StartToShoot          (shoot)
     *  2 = ShootToCollectFirst3  (collect)
     *  3 = ExtendCollect3        (collect)
     *  4 = BackShootFirst3       (shoot)
     *  5 = CollectSecond3        (collect)
     *  6 = BackShootSecond3      (shoot)
     *  7 = CollectThird3         (collect)
     *  8 = BackShootThird3       (shoot)
     *  9 = MoveForRP             (just drive)
     */
    private void startPath(int idx) {
        switch (idx) {
            case 1: follower.followPath(paths.StartToShoot);          break;
            case 2: follower.followPath(paths.ShootToCollectFirst3);  break;
            case 3: follower.followPath(paths.ExtendCollect3);        break;
            case 4: follower.followPath(paths.BackShootFirst3);       break;
            case 5: follower.followPath(paths.CollectSecond3);        break;
            case 6: follower.followPath(paths.BackShootSecond3);      break;
            case 7: follower.followPath(paths.CollectThird3);         break;
            case 8: follower.followPath(paths.BackShootThird3);       break;
            case 9: follower.followPath(paths.MoveForRP);             break;
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

    // ---------- Intake policy ----------
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

    // ---------- Gate control with RPM stability ----------
    private void updateGate() {
        try {
            double dist = distanceToShootPose();

            // Only attempt to open the gate during shoot phases
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
                panelsTelemetry.debug("Gate", "Opened (dist=" + String.format("%.2f", dist)
                        + ", rpmStable=" + rpmStable + ")");
            } else if (shouldBeClosed && !gateClosed && gateServo != null) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
                panelsTelemetry.debug("Gate", "Closed (dist=" + String.format("%.2f", dist) + ")");
            }

            panelsTelemetry.debug("Gate_dist", String.format("%.2f", dist));
            panelsTelemetry.debug("Gate_rpmErr", flywheel != null
                    ? String.format("%.1f", Math.abs(flywheel.getCurrentRPM() - flywheel.getTargetRPM()))
                    : "N/A");
            panelsTelemetry.debug("Gate_rpmStableT", String.format("%.2f", rpmStableTimer.getElapsedTimeSeconds()));
            panelsTelemetry.debug("Gate_inShoot", String.valueOf(inShootPhase));
            panelsTelemetry.debug("Gate_closed", String.valueOf(gateClosed));

        } catch (Exception e) {
            panelsTelemetry.debug("Gate", "updateGate error: " + e.getMessage());
        }
    }

    // ---------- Paths (Blue side — no mirroring) ----------
    public static class Paths {
        public PathChain StartToShoot;
        public PathChain ShootToCollectFirst3;
        public PathChain ExtendCollect3;
        public PathChain BackShootFirst3;
        public PathChain CollectSecond3;
        public PathChain BackShootSecond3;
        public PathChain CollectThird3;
        public PathChain BackShootThird3;
        public PathChain MoveForRP;

        public Paths(Follower follower) {
            StartToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(63.000, 8.000),
                            new Pose(58.000, 14.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            ShootToCollectFirst3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58.000, 14.000),
                            new Pose(49.367, 7.423),
                            new Pose(14.000, 22.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(245))
                    .build();

            ExtendCollect3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(14.000, 22.000),
                            new Pose(10.000, 16.000),
                            new Pose(13.163, 11.833)))
                    .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(270))
                    .build();

            BackShootFirst3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13.163, 11.833),
                            new Pose(58.000, 14.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                    .build();

            CollectSecond3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58.000, 14.000),
                            new Pose(56.042, 22.067),
                            new Pose(10.865, 21.577)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            BackShootSecond3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.865, 21.577),
                            new Pose(58.000, 14.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            // CollectThird3 — same geometry as CollectSecond3
            CollectThird3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58.000, 14.000),
                            new Pose(56.042, 22.067),
                            new Pose(10.865, 21.577)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // BackShootThird3 — same geometry as BackShootSecond3
            BackShootThird3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.865, 21.577),
                            new Pose(58.000, 14.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            MoveForRP = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(58.000, 14.000),
                            new Pose(31.684, 14.730)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}