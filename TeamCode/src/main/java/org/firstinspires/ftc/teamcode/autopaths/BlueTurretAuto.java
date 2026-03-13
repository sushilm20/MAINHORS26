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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.tracking.BearingTurretController;

@Autonomous(name = "Blue Turret Auto", group = "Autonomous", preselectTeleOp ="A HORS OFFICIAL ⭐")
@Configurable
public class BlueTurretAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState {
        IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, WAIT_FIRST_SHOOT,
        CLOSED_INTAKE_SEQUENCE, PRE_ACTION, INTAKE_RUN, CLAW_ACTION,
        WAIT_GATE_ALIGN, WAIT_GATE_CLEAR, FINISHED
    }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private Timer timedIntakeTimer;
    private boolean timedIntakeActive = false;

    private long clawActionStartMs = 0L;

    private Timer preActionTimer;
    private Timer poseWaitTimer;
    private Timer gateClearWaitTimer;
    private Timer firstShootWaitTimer;
    private Timer gateAlignWaitTimer;

    private boolean preActionTimerStarted = false;
    private boolean preActionEntered = false;
    private long shooterWaitStartMs = -1;

    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private DcMotor turretMotor;
    private DcMotor intakeMotor;

    private BNO055IMU pinpointImu = null;
    private BNO055IMU hubImu = null;
    private BNO055IMU imu = null;

    private FlywheelController flywheel;
    private BearingTurretController bearingTurretController;
    private static final double AUTO_SHOOTER_RPM = 2400;

    private Servo clawServo;
    private Servo rightHoodServo;
    private Servo gateServo;
    private boolean gateClosed = false;

    private long autoStartMs = -1;
    private boolean shutdownDone = false;

    // true only after start() so turret never tracks in init
    private boolean turretTrackingEnabled = false;

    // ── Velocity-based shoot gating ──
    private double lastPoseX = Double.NaN;
    private double lastPoseY = Double.NaN;
    private double lastPoseHeadingRad = Double.NaN;
    private long lastPoseTimeMs = -1L;
    private double translationalSpeedInPerS = 0.0;
    private double angularSpeedDegPerS = 0.0;
    private Timer speedStableTimer;
    private boolean speedStableTimerStarted = false;

    // ========================================
    // TIMING PARAMETERS (from BluePedroAuto)
    // ========================================
    @Sorter(sort = 0)  public static double INTAKE_RUN_SECONDS = 0.3;
    @Sorter(sort = 1)  public static double TIMED_INTAKE_SECONDS = 1.0;
    @Sorter(sort = 2)  public static double PRE_ACTION_FIRST_SHOOT_WAIT_SECONDS = 0.8;
    @Sorter(sort = 3)  public static double PRE_ACTION_WAIT_SECONDS = 1.0;
    @Sorter(sort = 4)  public static double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 1.5;
    @Sorter(sort = 5)  public static long SHOOTER_WAIT_TIMEOUT_MS = 1100L;

    // ========================================
    // INTAKE POWER SETTINGS (from BluePedroAuto)
    // ========================================
    @Sorter(sort = 10) public static double INTAKE_ON_POWER = -1.0;
    @Sorter(sort = 11) public static double SHOOT_POSE_INTAKE_POWER = -1.0;
    @Sorter(sort = 12) public static double CLOSED_INTAKE_POWER = -0.67;
    @Sorter(sort = 13) public static double CLOSED_INTAKE_TOLERANCE_IN = 10.0;
    @Sorter(sort = 14) public static double INTAKE_HOLD_POWER = -0.5;

    // ========================================
    // TOLERANCE SETTINGS (from BluePedroAuto)
    // ========================================
    @Sorter(sort = 20) public static double START_POSE_TOLERANCE_IN = 4.0;
    @Sorter(sort = 21) public static double SHOOT_MAX_TRANSLATIONAL_SPEED_IN_PER_S = 1.5;
    @Sorter(sort = 22) public static double SHOOT_MAX_ANGULAR_SPEED_DEG_PER_S = 20.0;
    @Sorter(sort = 23) public static double SPEED_STABLE_HOLD_SECONDS = 0.15;

    // ========================================
    // GATE SETTINGS (from BluePedroAuto)
    // ========================================
    @Sorter(sort = 30) public static double GATE_OPEN = 0.67;
    @Sorter(sort = 31) public static double GATE_CLOSED = 0.485;
    @Sorter(sort = 32) public static double GATE_OPEN_TOLERANCE_IN = 2.0;
    @Sorter(sort = 33) public static double GATE_CLOSE_TOLERANCE_IN = 4.0;
    @Sorter(sort = 34) public static double GATE_ALIGN_WAIT_SECONDS = 0.6;
    @Sorter(sort = 35) public static double WAIT_AFTER_GATE_CLEAR_SECONDS = 0.8;

    // ========================================
    // PATH POSES (from BluePedroAuto)
    // ========================================
    @Sorter(sort = 100) public static double START_X = 20.0;
    @Sorter(sort = 101) public static double START_Y = 122.0;
    @Sorter(sort = 102) public static double START_HEADING = 135.0;

    @Sorter(sort = 110) public static double SHOOT_POSE_X = 54;
    @Sorter(sort = 111) public static double SHOOT_POSE_Y = 84;
    @Sorter(sort = 112) public static double SHOOT_HEADING_INITIAL = 180;
    @Sorter(sort = 113) public static double SHOOT_HEADING_FIRST3 = 180;
    @Sorter(sort = 114) public static double SHOOT_SECOND3_HEADING = 180;
    @Sorter(sort = 115) public static double SHOOT_FINAL_HEADING = 180;

    @Sorter(sort = 120) public static double COLLECT_FIRST3_X = 20.0;
    @Sorter(sort = 121) public static double COLLECT_FIRST3_Y = 84.0;
    @Sorter(sort = 122) public static double COLLECT_FIRST3_HEADING = 180.0;

    @Sorter(sort = 125) public static double GATE_ALIGN_X = 24.0;
    @Sorter(sort = 126) public static double GATE_ALIGN_Y = 74.0;
    @Sorter(sort = 127) public static double GATE_ALIGN_HEADING = 180.0;

    @Sorter(sort = 130) public static double GATE_CLEAR_X = 20.0;
    @Sorter(sort = 131) public static double GATE_CLEAR_Y = 74.0;
    @Sorter(sort = 132) public static double GATE_CLEAR_HEADING = 180.0;

    @Sorter(sort = 140) public static double ALIGN_SECOND3_X = 50.0;
    @Sorter(sort = 141) public static double ALIGN_SECOND3_Y = 56.0;
    @Sorter(sort = 142) public static double ALIGN_SECOND3_HEADING = -180;

    @Sorter(sort = 150) public static double COLLECT_SECOND3_X = 12.0;
    @Sorter(sort = 151) public static double COLLECT_SECOND3_Y = 56.0;
    @Sorter(sort = 152) public static double COLLECT_SECOND3_HEADING = -180.0;

    @Sorter(sort = 160) public static double ALIGN_THIRD3_X = 50.0;
    @Sorter(sort = 161) public static double ALIGN_THIRD3_Y = 35.0;
    @Sorter(sort = 162) public static double ALIGN_THIRD3_HEADING = -180.0;

    @Sorter(sort = 170) public static double COLLECT_THIRD3_X = 12.0;
    @Sorter(sort = 171) public static double COLLECT_THIRD3_Y = 35.0;
    @Sorter(sort = 172) public static double COLLECT_THIRD3_HEADING = 180.0;

    @Sorter(sort = 180) public static double MOVE_RP_X = 36.0;
    @Sorter(sort = 181) public static double MOVE_RP_Y = 80.0;
    @Sorter(sort = 182) public static double MOVE_RP_HEADING = 135.0;

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
        firstShootWaitTimer = new Timer();
        speedStableTimer = new Timer();

        nextPathIndex = -1;
        preActionTimerStarted = false;
        preActionEntered = false;
        timedIntakeActive = false;
        speedStableTimerStarted = false;
        turretTrackingEnabled = false;

        lastPoseTimeMs = -1L;
        lastPoseX = Double.NaN;
        lastPoseY = Double.NaN;
        lastPoseHeadingRad = Double.NaN;

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
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // ensure 0 at init
                turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turretMotor.setPower(0.0); // no premovement
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Failed to map shooter/turret motors: " + e.getMessage());
        }

        try {
            try {
                pinpointImu = hardwareMap.get(BNO055IMU.class, "pinpoint");
            } catch (Exception e) {
                pinpointImu = null;
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
            if (shooterMotor != null) {
                flywheel = new FlywheelController(shooterMotor, shooterMotor2, telemetry);
                flywheel.setShooterOn(false);
            }

            if (turretMotor != null) {
                bearingTurretController = new BearingTurretController(turretMotor, follower, telemetry);
                bearingTurretController.zeroEncoder(); // keep encoder at 0 in init
                bearingTurretController.clearPid();
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Flywheel/BearingTurret init error: " + e.getMessage());
        }

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake mapping failed: " + e.getMessage());
        }

        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            if (clawServo != null) clawServo.setPosition(ClawController.CLAW_OPEN);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Claw servo mapping failed: " + e.getMessage());
        }

        try {
            rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
            if (rightHoodServo != null) rightHoodServo.setPosition(0.16);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Right hood servo mapping failed: " + e.getMessage());
        }

        try {
            gateServo = hardwareMap.get(Servo.class, "gateServo");
            if (gateServo != null) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Gate servo mapping failed: " + e.getMessage());
        }

        panelsTelemetry.debug("Status", "Initialized (turret BRAKE hold only in init)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // No turret tracking in init. Keep brake hold only.
        if (turretMotor != null) {
            turretMotor.setPower(0.0);
        }
    }

    @Override
    public void start() {
        autoStartMs = System.currentTimeMillis();
        turretTrackingEnabled = true; // starts tracking ONLY now

        if (flywheel != null) {
            flywheel.setShooterOn(true);
            flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
        }

        if (bearingTurretController != null) {
            bearingTurretController.clearPid(); // begin clean at start
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
            if (turretTrackingEnabled) {
                bearingTurretController.update(false, 0.0);
            } else if (turretMotor != null) {
                turretMotor.setPower(0.0);
            }
        }

        runStateMachine(nowMs);

        if (state != AutoState.WAIT_FIRST_SHOOT) {
            updateGate();
        }

        if (state != AutoState.IDLE && state != AutoState.FINISHED) {
            if (intakeMotor != null && intakeMotor.getPower() == 0.0) {
                intakeMotor.setPower(INTAKE_HOLD_POWER);
            }
        }

        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        if (turretMotor != null) {
            panelsTelemetry.debug("Turret Enc", turretMotor.getCurrentPosition());
            panelsTelemetry.debug("Turret Pwr", turretMotor.getPower());
            panelsTelemetry.debug("TurretTracking", turretTrackingEnabled);
        }
        panelsTelemetry.update(telemetry);

        if (state == AutoState.FINISHED && !shutdownDone) {
            resetToInitState();
            shutdownDone = true;
        }
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

    private void startIntake() { startIntake(INTAKE_ON_POWER); }

    private void startIntake(double power) {
        try { if (intakeMotor != null) intakeMotor.setPower(power); }
        catch (Exception e) { panelsTelemetry.debug("Intake", "startIntake error: " + e.getMessage()); }
    }

    private void stopIntake() {
        try { if (intakeMotor != null) intakeMotor.setPower(INTAKE_HOLD_POWER); }
        catch (Exception e) { panelsTelemetry.debug("Intake", "stopIntake error: " + e.getMessage()); }
    }

    private void fullStopIntake() {
        try { if (intakeMotor != null) intakeMotor.setPower(0.0); }
        catch (Exception e) { panelsTelemetry.debug("Intake", "fullStopIntake error: " + e.getMessage()); }
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

    private static double normalizeAngleRad(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
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

    private void runStateMachine(long nowMs) {
        if (timedIntakeActive) {
            if (timedIntakeTimer.getElapsedTimeSeconds() >= TIMED_INTAKE_SECONDS) {
                startIntake(INTAKE_ON_POWER);
                timedIntakeActive = false;
            }
        }

        switch (state) {
            case WAIT_FOR_SHOOTER:
                boolean atTarget = (flywheel != null && flywheel.isAtTarget());
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (System.currentTimeMillis() - shooterWaitStartMs);
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) startPath(1);
                break;

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

                        if (finished == 1) {
                            stopIntake();
                            if (gateServo != null && !gateClosed) {
                                gateServo.setPosition(GATE_CLOSED);
                                gateClosed = true;
                            }
                            firstShootWaitTimer.resetTimer();
                            state = AutoState.WAIT_FIRST_SHOOT;
                        } else {
                            state = AutoState.CLOSED_INTAKE_SEQUENCE;
                        }
                    } else {
                        int next = finished + 1;
                        if (next > 12) state = AutoState.FINISHED;
                        else startPath(next);
                    }
                }
                break;

            case WAIT_FIRST_SHOOT:
                if (firstShootWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_FIRST_SHOOT_WAIT_SECONDS) {
                    state = AutoState.CLOSED_INTAKE_SEQUENCE;
                }
                break;

            case WAIT_GATE_ALIGN:
                if (gateAlignWaitTimer.getElapsedTimeSeconds() >= GATE_ALIGN_WAIT_SECONDS) {
                    if (nextPathIndex > 0) { startPath(nextPathIndex); nextPathIndex = -1; }
                    else state = AutoState.FINISHED;
                }
                break;

            case WAIT_GATE_CLEAR:
                if (gateClearWaitTimer.getElapsedTimeSeconds() >= WAIT_AFTER_GATE_CLEAR_SECONDS) {
                    if (nextPathIndex > 0) { startPath(nextPathIndex); nextPathIndex = -1; }
                    else state = AutoState.FINISHED;
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
                        state = AutoState.PRE_ACTION;
                        speedStableTimerStarted = false;
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
                    if (nextPathIndex > 0 && nextPathIndex <= 12) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;

            case FINISHED:
            case IDLE:
            default:
                break;
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

    public static class Paths {
        public PathChain startToShoot, collectFirst3, gateAlign, gateClear, backToShootFirst3,
                alignToCollectSecond3, collectSecond3, backToShootSecond3,
                alignToCollectThird3, collectThird3, backToShootThird3, moveForRP;

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
                    .build();

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