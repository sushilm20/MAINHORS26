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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.tracking.TurretController;

@Autonomous(name = "Blue 12 Ball 🔷", group = "Autonomous", preselectTeleOp = "???HORS???")
@Configurable
public class ExperimentalBluePedroAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, CLOSED_INTAKE_SEQUENCE, PRE_ACTION, INTAKE_RUN, CLAW_ACTION, FINISHED }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private static final double INTAKE_RUN_SECONDS = 0.6; // reduced from 2.5

    private Timer timedIntakeTimer; // test
    private static final double TIMED_INTAKE_SECONDS = 1.0;
    private boolean timedIntakeActive = false;

    private long clawActionStartMs = 0L;
    private static final long CLAW_CLOSE_MS = 200L;

    private Timer preActionTimer;
    private static final double PRE_ACTION_WAIT_SECONDS = 0.2;

    private Timer poseWaitTimer;
    private static final double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.3;

    private boolean preActionTimerStarted = false;
    private boolean preActionEntered = false;

    private long shooterWaitStartMs = -1;
    private static final long SHOOTER_WAIT_TIMEOUT_MS = 1000L;

    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private DcMotor turretMotor;

    private BNO055IMU pinpointImu = null;
    private BNO055IMU hubImu = null;
    private BNO055IMU imu = null;

    private FlywheelController flywheel;
    private TurretController turretController;
    private static final double AUTO_SHOOTER_RPM = FlywheelController.TARGET_RPM_CLOSE;

    private DcMotor intakeMotor;
    private Servo leftCompressionServo;
    private Servo rightCompressionServo;

    private Servo clawServo;

    // Intake powers (negative = intake direction)
    private static double INTAKE_ON_POWER = -0.6;   // travel / non-shoot
    private static double SHOOT_POSE_INTAKE_POWER = -1.0; // at shoot pose
    private static double CLOSED_INTAKE_POWER = -0.6;     // pre-spin near shoot
    private static double CLOSED_INTAKE_TOLERANCE_IN = 12.0; // start pre-spin within 12"

    // Compression servos no longer used in the intake sequence
    private static final double LEFT_COMPRESSION_OFF = 0.5;
    private static final double RIGHT_COMPRESSION_OFF = 0.5;

    private int intakeSegmentEnd = -1; // retained but not used to stop intake

    // Primary shoot pose coordinates
    private static final double SHOOT_POSE_X = 68.0;
    private static final double SHOOT_POSE_Y = 78.0;
    private static final double START_POSE_TOLERANCE_IN = 6.0;

    private final boolean turretForceManualNoMove = false;

    private Servo gateServo;
    private boolean gateClosed = false;
    private static final double GATE_OPEN = 0.67;
    private static final double GATE_CLOSED = 0.5;

    // Gate control thresholds (open slightly earlier than pose tolerance, close as soon as out of range)
    private static final double GATE_OPEN_TOLERANCE_IN = START_POSE_TOLERANCE_IN + 3.0; // widened gate-open window
    private static final double GATE_CLOSE_TOLERANCE_IN = GATE_OPEN_TOLERANCE_IN + 1.0; // small hysteresis for close

    public ExperimentalBluePedroAuto() {}

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        // Start pose unchanged
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
            leftCompressionServo = hardwareMap.get(Servo.class, "leftCompressionServo");
            rightCompressionServo = hardwareMap.get(Servo.class, "rightCompressionServo");

            intakeMotor.setDirection(DcMotor.Direction.FORWARD);

            intakeMotor.setPower(0.0);
            if (leftCompressionServo != null) leftCompressionServo.setPosition(LEFT_COMPRESSION_OFF);
            if (rightCompressionServo != null) rightCompressionServo.setPosition(RIGHT_COMPRESSION_OFF);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake/compression mapping failed: " + e.getMessage());
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
        if (flywheel != null) {
            // flywheel.update(System.currentTimeMillis(), false);
        }
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
            panelsTelemetry.debug("LeftCompPos", leftCompressionServo != null ? leftCompressionServo.getPosition() : -1);
            panelsTelemetry.debug("RightCompPos", rightCompressionServo != null ? rightCompressionServo.getPosition() : -1);
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
            // compression servos intentionally not moved during intake sequence
        } catch (Exception e) {
            panelsTelemetry.debug("Intake", "startIntake error: " + e.getMessage());
        }
    }

    private void stopIntake() {
        try {
            if (intakeMotor != null) intakeMotor.setPower(0.0);
            // leave compression servos untouched
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

        // Always run intake while traveling (non-shoot pose)
        startIntake(INTAKE_ON_POWER);

        if (idx == 4 || idx == 7 || idx == 10) {
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
                // keep intake on at travel power instead of stopping
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
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) {
                    startPath(1);
                }
                break;

            case RUNNING_PATH:
                if (!follower.isBusy()) {
                    int finished = currentPathIndex;

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
                // Pre-spin intake (gate stays closed until gate tolerance hit)
                double distPre = distanceToShootPose();
                if (distPre <= CLOSED_INTAKE_TOLERANCE_IN) {
                    startIntake(CLOSED_INTAKE_POWER);
                }
                // Transition to PRE_ACTION once within main pose tolerance
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
                        // At shoot pose: full intake power
                        startIntake(SHOOT_POSE_INTAKE_POWER);
                        intakeTimer.resetTimer();
                        state = AutoState.INTAKE_RUN;
                    }
                }
                break;

            case INTAKE_RUN:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                    // After shoot intake window, go back to travel intake power
                    startIntake(INTAKE_ON_POWER);
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

    /**
     * Gate behavior:
     * - Open quickly when within GATE_OPEN_TOLERANCE_IN of the shoot pose.
     * - Close immediately once outside GATE_CLOSE_TOLERANCE_IN.
     * This keeps the gate closed whenever we're not effectively at the shoot state.
     */
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
            // Path1: start -> primary shoot pose (68,78,180)
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 122.000, Math.toRadians(135)), new Pose(68.000, 78.000, Math.toRadians(180)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            // Path2: shoot -> (20,80,175)
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(68.000, 78.000, Math.toRadians(180)), new Pose(20.000, 80.000, Math.toRadians(175)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(175))
                    .build();

            // Path3: (20,80,175) -> (12,76,90) gate clear
            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 80.000, Math.toRadians(175)), new Pose(16.0, 76.000, Math.toRadians(90)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(90))
                    .build();

            // Path4: (12,76,90) -> (68,78,-146) angled shoot
            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.0, 76.000, Math.toRadians(90)), new Pose(68.000, 78.000, Math.toRadians(-146)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-146))
                    .build();

            // Path5: (68,78,-146) -> (44,57,-175)
            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(68.000, 78.000, Math.toRadians(-146)), new Pose(44.000, 57.000, Math.toRadians(-175)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-146), Math.toRadians(-175))
                    .build();

            // Path6: (44,57,-175) -> (12,56,-180)
            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.000, 57.000, Math.toRadians(-175)), new Pose(12.000, 56.000, Math.toRadians(-180)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-175), Math.toRadians(-180))
                    .build();

            // Path7: (12,56,-180) -> (68,78,-125) angled shoot
            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.000, 56.000, Math.toRadians(-180)), new Pose(68.000, 78.000, Math.toRadians(-125)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-125))
                    .build();

            // Path8: (68,78,-125) -> (47,33,-180) alignment before collect 3
            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(68.000, 78.000, Math.toRadians(-125)), new Pose(47.000, 33.000, Math.toRadians(-180)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-125), Math.toRadians(-180))
                    .build();

            // Path9: (47,33,-180) -> (12,33,180)
            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.000, 33.000, Math.toRadians(-180)), new Pose(12.000, 33.000, Math.toRadians(180)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(180))
                    .build();

            // Path10: (12,33,180) -> (68,78,180) final shoot pose
            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.000, 33.000, Math.toRadians(180)), new Pose(68.000, 78.000, Math.toRadians(180)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path11: hold/adjust at final shoot pose
            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(68.000, 78.000, Math.toRadians(180)), new Pose(68.000, 78.000, Math.toRadians(180)))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}