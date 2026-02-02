package org.firstinspires.ftc.teamcode.autopaths;

// (all your imports — unchanged)
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

@Autonomous(name = "A Wild Experiment 12 Ball 🔷", group = "Autonomous", preselectTeleOp = "HORS EXPERIMENTAL 🤖")
@Configurable
public class ExperimentalBluePedroAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, CLOSED_INTAKE_SEQUENCE, PRE_ACTION, INTAKE_RUN, CLAW_ACTION, WAIT_GATE_ALIGN, WAIT_GATE_CLEAR, FINISHED }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private Timer timedIntakeTimer;
    private boolean timedIntakeActive = false;

    private long clawActionStartMs = 0L;
    private Timer preActionTimer;
    private Timer poseWaitTimer;
    private Timer gateAlignWaitTimer;
    private Timer gateClearWaitTimer;
    private boolean preActionTimerStarted = false;
    private boolean preActionEntered = false;
    private long shooterWaitStartMs = -1;

    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private DcMotor turretMotor;

    private BNO055IMU pinpointImu = null;
    private BNO055IMU hubImu = null;
    private BNO055IMU imu = null;

    private FlywheelController flywheel;
    private TurretController turretController;
    private static final double AUTO_SHOOTER_RPM = 2650;

    private DcMotor intakeMotor;
    private Servo clawServo;
    private Servo rightHoodServo;
    private Servo gateServo;
    private boolean gateClosed = false;

    private int intakeSegmentEnd = -1;
    private final boolean turretForceManualNoMove = false;

    // Timing/telemetry helpers
    private long autoStartMs = -1;
    private boolean shutdownDone = false;

    // Turret targets for this auto
    private int turretHoldTarget = 0;
    private boolean turretForceHold = true;

    // ========================================
    // TIMING PARAMETERS
    // ========================================
    @Sorter(sort = 0)
    public static double INTAKE_RUN_SECONDS = 0.7;
    @Sorter(sort = 1)
    public static double TIMED_INTAKE_SECONDS = 1.1;
    @Sorter(sort = 2)
    public static long CLAW_CLOSE_MS = 190L;
    @Sorter(sort = 3)
    public static double PRE_ACTION_WAIT_SECONDS = 0.4;
    @Sorter(sort = 4)
    public static double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.9;
    @Sorter(sort = 5)
    public static long SHOOTER_WAIT_TIMEOUT_MS = 1100L;
    // ========================================
    // INTAKE POWER SETTINGS
    // ========================================
    @Sorter(sort = 10)
    public static double INTAKE_ON_POWER = -0.75;
    @Sorter(sort = 11)
    public static double SHOOT_POSE_INTAKE_POWER = -1.0;
    @Sorter(sort = 12)
    public static double CLOSED_INTAKE_POWER = -0.67;
    @Sorter(sort = 13)
    public static double CLOSED_INTAKE_TOLERANCE_IN = 9.0;
    // ========================================
    // TOLERANCE SETTINGS
    // ========================================
    @Sorter(sort = 20)
    public static double START_POSE_TOLERANCE_IN = 5.0;
    // ========================================
    // GATE SETTINGS
    // ========================================
    @Sorter(sort = 30)
    public static double GATE_OPEN = 0.67;
    @Sorter(sort = 31)
    public static double GATE_CLOSED = 0.45;
    @Sorter(sort = 32)
    public static double GATE_OPEN_TOLERANCE_IN = 3.0;
    @Sorter(sort = 33)
    public static double GATE_CLOSE_TOLERANCE_IN = 7.0;
    @Sorter(sort = 34)
    public static double GATE_ALIGN_WAIT_SECONDS = 0.5;
    @Sorter(sort = 35)
    public static double WAIT_AFTER_GATE_CLEAR_SECONDS = 1.0;
    // ========================================
    // PATH POSES - START POSITION
    // ========================================
    @Sorter(sort = 100)
    public static double START_X = 20.0;
    @Sorter(sort = 101)
    public static double START_Y = 122.0;
    @Sorter(sort = 102)
    public static double START_HEADING = 135.0;
    // ========================================
    // PATH POSES - SHOOT POSITION (Primary)
    // ========================================
    @Sorter(sort = 110)
    public static double SHOOT_POSE_X = 58.0;
    @Sorter(sort = 111)
    public static double SHOOT_POSE_Y = 80.0;
    @Sorter(sort = 112)
    public static double SHOOT_HEADING_INITIAL = 180.0;
    @Sorter(sort = 113)
    public static double SHOOT_HEADING_FIRST3 = 180.0;
    @Sorter(sort = 114)
    public static double SHOOT_SECOND3_HEADING = 180.0;
    @Sorter(sort = 115)
    public static double SHOOT_FINAL_HEADING = 180.0;
    // ========================================
    // PATH POSES - COLLECT FIRST 3 POSITION
    // ========================================
    @Sorter(sort = 120)
    public static double COLLECT_FIRST3_X = 21.0;
    @Sorter(sort = 121)
    public static double COLLECT_FIRST3_Y = 80.0;
    @Sorter(sort = 122)
    public static double COLLECT_FIRST3_HEADING = 175.0;
    // ========================================
    // PATH POSES - GATE ALIGN POSITION
    // ========================================
    @Sorter(sort = 125)
    public static double GATE_ALIGN_X = 24.0;
    @Sorter(sort = 126)
    public static double GATE_ALIGN_Y = 74.0;
    @Sorter(sort = 127)
    public static double GATE_ALIGN_HEADING = 179.0;
    // ========================================
    // PATH POSES - GATE CLEAR POSITION
    // ========================================
    @Sorter(sort = 130)
    public static double GATE_CLEAR_X = 20.0;
    @Sorter(sort = 131)
    public static double GATE_CLEAR_Y = 74.0;
    @Sorter(sort = 132)
    public static double GATE_CLEAR_HEADING = 179.0;
    // ========================================
    // PATH POSES - ALIGN SECOND 3 POSITION
    // ========================================
    @Sorter(sort = 140)
    public static double ALIGN_SECOND3_X = 50.0;
    @Sorter(sort = 141)
    public static double ALIGN_SECOND3_Y = 58.0;
    @Sorter(sort = 142)
    public static double ALIGN_SECOND3_HEADING = -175.0;
    // ========================================
    // PATH POSES - COLLECT SECOND 3 POSITION
    // ========================================
    @Sorter(sort = 150)
    public static double COLLECT_SECOND3_X = 14.0;
    @Sorter(sort = 151)
    public static double COLLECT_SECOND3_Y = 58.0;
    @Sorter(sort = 152)
    public static double COLLECT_SECOND3_HEADING = -180.0;
    // ========================================
    // PATH POSES - ALIGN THIRD 3 POSITION
    // ========================================
    @Sorter(sort = 160)
    public static double ALIGN_THIRD3_X = 50.0;
    @Sorter(sort = 161)
    public static double ALIGN_THIRD3_Y = 35.0;
    @Sorter(sort = 162)
    public static double ALIGN_THIRD3_HEADING = -180.0;
    // ========================================
    // PATH POSES - COLLECT THIRD 3 POSITION
    // ========================================
    @Sorter(sort = 170)
    public static double COLLECT_THIRD3_X = 14.0;
    @Sorter(sort = 171)
    public static double COLLECT_THIRD3_Y = 35.0;
    @Sorter(sort = 172)
    public static double COLLECT_THIRD3_HEADING = 180.0;
    // ========================================
    // PATH POSES - MOVE FOR RP POSITION
    // ========================================
    @Sorter(sort = 180)
    public static double MOVE_RP_X = 36.0;
    @Sorter(sort = 181)
    public static double MOVE_RP_Y = 80.0;
    @Sorter(sort = 182)
    public static double MOVE_RP_HEADING = 135.0;

    public ExperimentalBluePedroAuto() {}

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
        nextPathIndex = -1;
        intakeSegmentEnd = -1;
        preActionTimerStarted = false;
        preActionEntered = false;
        timedIntakeActive = false;

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
                pinpointImu = null;
                panelsTelemetry.debug("Init", "PinPoint IMU 'pinpoint' not found: " + e.getMessage());
            }

            try {
                hubImu = hardwareMap.get(BNO055IMU.class, "imu");
                panelsTelemetry.debug("Init", "Found expansion hub IMU as 'imu'");
            } catch (Exception e) {
                hubImu = null;
                panelsTelemetry.debug("Init", "Expansion hub IMU 'imu' not found: " + e.getMessage());
            }

            imu = (pinpointImu != null) ? pinpointImu : hubImu;

            if (imu != null) {
                BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
                imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                imu.initialize(imuParams);
                panelsTelemetry.debug("Init", "IMU initialized (using " + ((pinpointImu != null) ? "PinPoint" : "Expansion Hub") + ")");
            } else {
                panelsTelemetry.debug("Init", "No IMU found (neither 'pinpoint' nor 'imu'). Turret will not have heading data.");
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "IMU not found or failed to init: " + e.getMessage());
        }

        try {
            if (shooterMotor != null) flywheel = new FlywheelController(shooterMotor, shooterMotor2, telemetry);
            if (turretMotor != null) {
                turretController = new TurretController(turretMotor, imu, telemetry);
            }
            if (turretController != null) {
                turretController.captureReferences();
                turretController.resetPidState();
            }
            if (flywheel != null) {
                flywheel.setShooterOn(false);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Flywheel/TurretController creation error: " + e.getMessage());
        }
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake Motor mapping failed: " + e.getMessage());
        }
        try {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            if (clawServo != null) {
                clawServo.setPosition(0.63);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Claw servo mapping failed: " + e.getMessage());
        }
        try {
            rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
            if (rightHoodServo != null) {
                rightHoodServo.setPosition(0.16);
                panelsTelemetry.debug("Init", "Right hood servo initialized to 0.16");
            }
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

        panelsTelemetry.debug("Status", "Initialized (shooter remains OFF until start())");
        panelsTelemetry.update(telemetry);

        turretHoldTarget = 0;
    }

    @Override
    public void init_loop() {
        if (flywheel != null) {
            // flywheel.update(System.currentTimeMillis(), false);
        }
        if (turretController != null && turretForceHold) {
            turretController.holdPositionTicks(turretHoldTarget);
        }
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

        // Turn turret before any paths run
        turretHoldTarget = 230;   // target position for turret
        turretForceHold = true;

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
        if (turretController != null && turretForceHold) {
            turretController.holdPositionTicks(turretHoldTarget);
        }

        runStateMachine(nowMs);
        updateGate();

        double elapsedSec = (autoStartMs > 0) ? (nowMs - autoStartMs) / 1000.0 : 0.0;
        panelsTelemetry.debug("Elapsed(s)", String.format("%.2f", elapsedSec));
        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        if (flywheel != null) {
            panelsTelemetry.debug("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
            panelsTelemetry.debug("Fly Target", String.format("%.1f", flywheel.getTargetRPM()));
            panelsTelemetry.debug("Fly On", flywheel.isShooterOn());
        }
        if (turretMotor != null && turretController != null) {
            panelsTelemetry.debug("Turret Enc", turretMotor.getCurrentPosition());
            panelsTelemetry.debug("Turret Power", turretController.getLastAppliedPower());
            panelsTelemetry.debug("TurretTrackingEnabled", String.valueOf(!turretForceManualNoMove));
            panelsTelemetry.debug("TurretHoldingEncoder", String.valueOf(turretForceHold));
        }
        if (intakeMotor != null) {
            panelsTelemetry.debug("Intake Power", intakeMotor.getPower());
        }
        if (clawServo != null) {
            panelsTelemetry.debug("ClawPos", clawServo.getPosition());
        }
        double dist = distanceToShootPose();
        panelsTelemetry.debug("DistToShootPose", String.format("%.2f", dist));
        panelsTelemetry.debug("GateClosed", String.valueOf(gateClosed));

        if (imu == pinpointImu && pinpointImu != null) {
            panelsTelemetry.debug("IMU Source", "PinPoint ('pinpoint')");
        } else if (imu == hubImu && hubImu != null) {
            panelsTelemetry.debug("IMU Source", "Expansion Hub ('imu')");
        } else {
            panelsTelemetry.debug("IMU Source", "None");
        }
        panelsTelemetry.update(telemetry);

        if (state == AutoState.FINISHED && !shutdownDone) {
            resetToInitState();
            shutdownDone = true;
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
        if (clawServo != null) {
            clawServo.setPosition(0.63);
        }
        if (rightHoodServo != null) {
            rightHoodServo.setPosition(0.16);
        }
        if (turretController != null && turretForceHold) {
            turretHoldTarget = 0;
            turretController.holdPositionTicks(turretHoldTarget);
        }
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
        try { if (intakeMotor != null) intakeMotor.setPower(0.0); }
        catch (Exception e) { panelsTelemetry.debug("Intake", "stopIntake error: " + e.getMessage()); }
    }
    private boolean endsAtShoot(int pathIndex) { return pathIndex == 1 || pathIndex == 5 || pathIndex == 8 || pathIndex == 11; }
    private double distanceToShootPose() {
        try {
            Pose p = follower.getPose();
            double dx = p.getX() - SHOOT_POSE_X;
            double dy = p.getY() - SHOOT_POSE_Y;
            return Math.hypot(dx, dy);
        } catch (Exception e) { return Double.POSITIVE_INFINITY; }
    }

    private void startPath(int idx) {
        if (idx < 1 || idx > 12) { currentPathIndex = 0; state = AutoState.FINISHED; return; }
        startIntake(INTAKE_ON_POWER);
        if (idx == 5 || idx == 8 || idx == 11) {
            timedIntakeTimer.resetTimer();
            timedIntakeActive = true;
            panelsTelemetry.debug("TimedIntake", "Started timed intake for path " + idx);
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
                panelsTelemetry.debug("TimedIntake", "Timed intake period done; continuing at travel power");
            } else {
                panelsTelemetry.debug("TimedIntake", String.format("remaining=%.2fs", TIMED_INTAKE_SECONDS - timedIntakeTimer.getElapsedTimeSeconds()));
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
                    // turret turning now happens before paths start; no longer adjust after collectFirst3
                    if (finished == 3) { gateAlignWaitTimer.resetTimer(); nextPathIndex = 4; state = AutoState.WAIT_GATE_ALIGN; break; }
                    if (finished == 4) { gateClearWaitTimer.resetTimer(); nextPathIndex = 5; state = AutoState.WAIT_GATE_CLEAR; break; }
                    if (endsAtShoot(finished)) {
                        nextPathIndex = finished + 1;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        state = AutoState.CLOSED_INTAKE_SEQUENCE;
                    } else {
                        int next = finished + 1;
                        if (next > 12) { state = AutoState.FINISHED; }
                        else { startPath(next); }
                    }
                }
                break;
            case WAIT_GATE_ALIGN:
                if (gateAlignWaitTimer.getElapsedTimeSeconds() >= GATE_ALIGN_WAIT_SECONDS) {
                    if (nextPathIndex > 0) { startPath(nextPathIndex); nextPathIndex = -1; }
                    else { state = AutoState.FINISHED; }
                }
                break;
            case WAIT_GATE_CLEAR:
                if (gateClearWaitTimer.getElapsedTimeSeconds() >= WAIT_AFTER_GATE_CLEAR_SECONDS) {
                    if (nextPathIndex > 0) { startPath(nextPathIndex); nextPathIndex = -1; }
                    else { state = AutoState.FINISHED; }
                }
                break;
            case CLOSED_INTAKE_SEQUENCE:
                double distPre = distanceToShootPose();
                if (distPre <= CLOSED_INTAKE_TOLERANCE_IN) { startIntake(CLOSED_INTAKE_POWER); }
                if (distPre <= START_POSE_TOLERANCE_IN) { state = AutoState.PRE_ACTION; }
                break;
            case PRE_ACTION:
                if (!preActionEntered) {
                    poseWaitTimer.resetTimer();
                    preActionTimerStarted = false;
                    preActionEntered = true;
                    panelsTelemetry.debug("PRE_ACTION", "Entered PRE_ACTION, starting pose-wait");
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
                        startIntake(SHOOT_POSE_INTAKE_POWER);
                        intakeTimer.resetTimer();
                        state = AutoState.INTAKE_RUN;
                    }
                }
                break;
            case INTAKE_RUN:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                    startIntake(INTAKE_ON_POWER);
                    flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                    if (clawServo != null) clawServo.setPosition(0.2);
                    clawActionStartMs = System.currentTimeMillis();
                    state = AutoState.CLAW_ACTION;
                }
                break;
            case CLAW_ACTION:
                if (System.currentTimeMillis() >= clawActionStartMs + CLAW_CLOSE_MS) {
                    if (clawServo != null) clawServo.setPosition(0.63);
                    if (nextPathIndex > 0 && nextPathIndex <= 12) { startPath(nextPathIndex); nextPathIndex = -1; }
                    else { state = AutoState.FINISHED; }
                }
                break;
            case FINISHED:
                if (turretForceHold) { turretHoldTarget = 0; }
                break;
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
                panelsTelemetry.debug("Gate", "Opened (dist=" + String.format("%.2f", dist) + ")");
            } else if (dist >= GATE_CLOSE_TOLERANCE_IN && gateServo != null && !gateClosed) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
                panelsTelemetry.debug("Gate", "Closed (dist=" + String.format("%.2f", dist) + ")");
            }
        } catch (Exception e) { panelsTelemetry.debug("Gate", "updateGate error: " + e.getMessage()); }
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
            // ---- Start to Primary Shoot Pose ----
            startToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(START_X, START_Y),
                            new Pose(SHOOT_POSE_X, SHOOT_POSE_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(START_HEADING), Math.toRadians(SHOOT_HEADING_INITIAL))
                    .build();

            // ---- Shoot to Collect First 3 ----
            collectFirst3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                            new Pose(COLLECT_FIRST3_X, COLLECT_FIRST3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_INITIAL), Math.toRadians(COLLECT_FIRST3_HEADING))
                    .build();

            // ---- Collect First 3 to Gate Align ----
            gateAlign = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(COLLECT_FIRST3_X, COLLECT_FIRST3_Y),
                            new Pose(GATE_ALIGN_X, GATE_ALIGN_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_FIRST3_HEADING), Math.toRadians(GATE_ALIGN_HEADING))
                    .build();

            // ---- Gate Align to Gate Clear ----
            gateClear = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(GATE_ALIGN_X, GATE_ALIGN_Y),
                            new Pose(GATE_CLEAR_X, GATE_CLEAR_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(GATE_ALIGN_HEADING), Math.toRadians(GATE_CLEAR_HEADING))
                    .build();

            // ---- Gate Clear to Shoot (for First 3) ----
            backToShootFirst3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(GATE_CLEAR_X, GATE_CLEAR_Y),
                            new Pose(SHOOT_POSE_X, SHOOT_POSE_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(GATE_CLEAR_HEADING), Math.toRadians(SHOOT_HEADING_FIRST3))
                    .build();

            // ---- Shoot to Align for Second 3 ----
            alignToCollectSecond3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                            new Pose(ALIGN_SECOND3_X, ALIGN_SECOND3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_FIRST3), Math.toRadians(ALIGN_SECOND3_HEADING))
                    .build();

            // ---- Align to Collect Second 3 ----
            collectSecond3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(ALIGN_SECOND3_X, ALIGN_SECOND3_Y),
                            new Pose(COLLECT_SECOND3_X, COLLECT_SECOND3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(ALIGN_SECOND3_HEADING), Math.toRadians(COLLECT_SECOND3_HEADING))
                    .build();

            // ---- Collect Second 3 to Shoot ----
            backToShootSecond3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(COLLECT_SECOND3_X, COLLECT_SECOND3_Y),
                            new Pose(SHOOT_POSE_X, SHOOT_POSE_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_SECOND3_HEADING), Math.toRadians(SHOOT_SECOND3_HEADING))
                    .build();

            // ---- Shoot to Align for Third 3 ----
            alignToCollectThird3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                            new Pose(ALIGN_THIRD3_X, ALIGN_THIRD3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_SECOND3_HEADING), Math.toRadians(ALIGN_THIRD3_HEADING))
                    .build();

            // ---- Align to Collect Third 3 ----
            collectThird3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(ALIGN_THIRD3_X, ALIGN_THIRD3_Y),
                            new Pose(COLLECT_THIRD3_X, COLLECT_THIRD3_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(ALIGN_THIRD3_HEADING), Math.toRadians(COLLECT_THIRD3_HEADING))
                    .build();

            // ---- Collect Third 3 to Shoot ----
            backToShootThird3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(COLLECT_THIRD3_X, COLLECT_THIRD3_Y),
                            new Pose(SHOOT_POSE_X, SHOOT_POSE_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(COLLECT_THIRD3_HEADING), Math.toRadians(SHOOT_FINAL_HEADING))
                    .build();

            // ---- Shoot to RP Move ----
            moveForRP = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                            new Pose(MOVE_RP_X, MOVE_RP_Y)))
                    .setLinearHeadingInterpolation(Math.toRadians(SHOOT_FINAL_HEADING), Math.toRadians(MOVE_RP_HEADING))
                    .build();
        }
    }
}