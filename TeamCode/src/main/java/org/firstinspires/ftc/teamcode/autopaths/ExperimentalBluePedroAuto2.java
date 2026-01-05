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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

import static org.firstinspires.ftc.teamcode.autopaths.AutoVarsBlue.Shared.*;

@Autonomous(name = "Config 12 ball ", group = "Autonomous", preselectTeleOp = "???HORS???")
@Configurable
public class ExperimentalBluePedroAuto2 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, CLOSED_INTAKE_SEQUENCE, PRE_ACTION, INTAKE_RUN, CLAW_ACTION, FINISHED }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;

    private Timer timedIntakeTimer;//test
    private boolean timedIntakeActive = false;

    private long clawActionStartMs = 0L;

    private Timer preActionTimer;

    private Timer poseWaitTimer;

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

    private DcMotor intakeMotor;

    private Servo clawServo;

    private int intakeSegmentEnd = -1;

    private final boolean turretForceManualNoMove = false;

    private Servo gateServo;
    private boolean dpadUpLast = false;
    private boolean gateClosed = false;

    public ExperimentalBluePedroAuto2() {}

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        follower.setStartingPose(new Pose(20, 122, Math.toRadians(135)));

        intakeTimer = new Timer();
        timedIntakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        nextPathIndex = -1;
        intakeSegmentEnd = -1;
        preActionTimerStarted = false;
        preActionEntered = false;
        timedIntakeActive = false;

        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
            shooterMotor2 = hardwareMap.get(DcMotor.class, "shooter2");
            turretMotor = hardwareMap.get(DcMotor.class, "turret");

            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            if (shooterMotor2 != null) {
                shooterMotor2.setDirection(DcMotor.Direction.FORWARD); // mirror of shooter
                shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            turretMotor.setDirection(DcMotor.Direction.FORWARD);

            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake mapping failed: " + e.getMessage());
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
    }

    @Override
    public void init_loop() {
        if (turretController != null) {
            turretController.update(false, 0.0);
        }
    }

    @Override
    public void start() {
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

        if (flywheel != null) {
            flywheel.handleLeftTrigger(false);
            flywheel.update(nowMs, false);
        }

        // Mirror shooter power to secondary motor
        if (shooterMotor != null && shooterMotor2 != null) {
            try {
                shooterMotor2.setPower(shooterMotor.getPower());
            } catch (Exception e) {
                panelsTelemetry.debug("Shooter2", "Power mirror error: " + e.getMessage());
            }
        }

        if (turretController != null) {
            turretController.update(false, 0.0);
        }

        runStateMachine(nowMs);

        updateGate();

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
            panelsTelemetry.debug("TurretTrackingEnabled", String.valueOf(!turretForceManualNoMove));
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
    }

    @Override
    public void stop() {
        if (flywheel != null) {
            flywheel.setShooterOn(false);
            flywheel.update(System.currentTimeMillis(), false);
        }
        if (turretController != null) {
            turretController.update(false, 0.0);
        }

        stopIntake();
        if (clawServo != null) clawServo.setPosition(0.63);

        if (gateServo != null) {
            gateServo.setPosition(GATE_CLOSED);
            gateClosed = true;
        }

        state = AutoState.FINISHED;
    }

    private void startIntake() {
        startIntake(INTAKE_ON_POWER);
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

    private boolean endsAtShoot(int pathIndex) {
        return pathIndex == 1 || pathIndex == 4 || pathIndex == 7 || pathIndex == 10;
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
        if (idx < 1 || idx > 11) {
            currentPathIndex = 0;
            state = AutoState.FINISHED;
            return;
        }

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

        if (idx == 4 || idx == 7 || idx == 10) {
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
        if (timedIntakeActive) {
            if (timedIntakeTimer.getElapsedTimeSeconds() >= TIMED_INTAKE_SECONDS) {
                stopIntake();
                timedIntakeActive = false;
                intakeSegmentEnd = -1;
                panelsTelemetry.debug("TimedIntake", "Timed intake ended after " + TIMED_INTAKE_SECONDS + "s");
            } else {
                panelsTelemetry.debug("TimedIntake", String.format("remaining=%.2fs", TIMED_INTAKE_SECONDS - timedIntakeTimer.getElapsedTimeSeconds()));
            }
        }

        switch (state) {
            case WAIT_FOR_SHOOTER:
                boolean atTarget = (flywheel != null && flywheel.isAtTarget());
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (System.currentTimeMillis() - shooterWaitStartMs);
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) {
                    startPath(1);
                }
                break;

            case RUNNING_PATH:
                if (!follower.isBusy()) {
                    int finished = currentPathIndex;
                    if (intakeSegmentEnd == finished) {
                        stopIntake();
                        intakeSegmentEnd = -1;
                    }

                    if (endsAtShoot(finished)) {
                        nextPathIndex = finished + 1;
                        preActionTimerStarted = false;
                        preActionEntered = false;
                        state = AutoState.CLOSED_INTAKE_SEQUENCE;
                    } else {
                        int next = finished + 1;
                        if (next > 11) {
                            state = AutoState.FINISHED;
                        } else {
                            startPath(next);
                        }
                    }
                }
                break;

            case CLOSED_INTAKE_SEQUENCE:
                double distPre = distanceToShootPose();
                if (distPre <= CLOSED_INTAKE_TOLERANCE_IN) {
                    startIntake(CLOSED_INTAKE_POWER);
                }
                if (distPre <= START_POSE_TOLERANCE_IN) {
                    state = AutoState.PRE_ACTION;
                }
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
                    if (intakeSegmentEnd == -1) {
                        stopIntake();
                    }
                    flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                    if (clawServo != null) clawServo.setPosition(0.2); // close
                    clawActionStartMs = System.currentTimeMillis();
                    state = AutoState.CLAW_ACTION;
                }
                break;

            case CLAW_ACTION:
                if (System.currentTimeMillis() >= clawActionStartMs + CLAW_CLOSE_MS) {
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
        } catch (Exception e) {
            panelsTelemetry.debug("Gate", "updateGate error: " + e.getMessage());
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
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(44.000, 82.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 82.000), new Pose(24.000, 82.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 82.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(46.000, 57.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(46.000, 57.000), new Pose(18.00, 57.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.00, 57.000), new Pose(34.000, 70.000))
                    )
                    .setNoDeceleration()
                    .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                    .addPath(
                            new BezierLine(new Pose(34.000, 70.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(45.000, 33.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.000, 33.000), new Pose(15.000, 33.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 33.000), new Pose(48.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(40.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }
}
