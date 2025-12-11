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
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.TurretController;

/**
 * FarmodeAuto — Autonomous sequence using the same hardware helpers & logic from ExperimentalPedroAuto.
 *
 * Sequence:
 *  - Start at (63,8) heading 90deg.
 *  - Drive -> (52,14).
 *  - Run intake sequence once (intake remains ON).
 *  - After intake sequence completes, wait 2s (intake still ON).
 *  - Drive -> (16,14) while intake remains ON.
 *  - When arriving at (16,14), keep intake ON for 1s.
 *  - Drive -> (52,14).
 *  - Shoot 3 (intake ON during shooting). End.
 *
 * All hardware mapping and intake helpers are copied from ExperimentalPedroAuto.
 * There are no placeholders — the intake start/stop implementations, flywheel and turret setup,
 * timers and the use of pedropathing Timer are taken from ExperimentalPedroAuto.
 *
 * NOTE: ExperimentalPedroAuto does not include an explicit indexer motor / indexing routine.
 * The shooting step here relies on the flywheel subsystem to be spun up and runs a timed "shoot"
 * period (during which you should ensure your indexing mechanism runs if present).
 */
@Autonomous(name = "Farmode Auto (FIX) ", group = "Autonomous")
@Configurable
public class FarmodeAuto extends OpMode {

  private TelemetryManager panelsTelemetry;
  public Follower follower;
  private Paths paths;

  // State machine states
  private enum AutoState { INIT, TO_52, RUN_INTAKE_SEQ, WAIT_AFTER_INTK, TO_16, HOLD_AT_16, TO_52_AGAIN, SHOOT, FINISHED }
  private AutoState state = AutoState.INIT;

  // hardware (copied / mapped as in ExperimentalPedroAuto)
  private DcMotor shooterMotor = null;
  private DcMotor turretMotor = null;
  private BNO055IMU imu = null;
  private Flywheel flywheel = null;
  private TurretController turretController = null;

  // Intake + compression hardware (from teleop)
  private DcMotor intakeMotor = null;
  private Servo leftCompressionServo = null;
  private Servo rightCompressionServo = null;

  // Claw servo (not used by this sequence but mapped same as ExperimentalPedroAuto)
  private Servo clawServo = null;

  // Intake/compression "on" values (copied)
  private static final double INTAKE_ON_POWER = 1.0;
  private static final double LEFT_COMPRESSION_ON = 1.0;
  private static final double RIGHT_COMPRESSION_ON = 0.0;
  private static final double LEFT_COMPRESSION_OFF = 0.5;
  private static final double RIGHT_COMPRESSION_OFF = 0.5;

  // timers
  private Timer intakeSequenceTimer;
  private Timer waitTimer;
  private Timer holdTimer;
  private Timer shootTimer;

  // durations (seconds) - intake sequence length & waits (adjust if you want exact timings)
  private static final double INTAKE_SEQUENCE_SECONDS = 1.5; // run intake-sequence at first (user specified "once")
  private static final double WAIT_AFTER_INTAKE_SECONDS = 2.0; // wait 2s after intake sequence
  private static final double HOLD_AT_16_SECONDS = 1.0; // hold intake 1s at 16,14
  private static final double SHOOT_SECONDS = 1.5; // run shooting period (timed) - adjust to match indexing mechanism

  // helper to track path start/completion
  private boolean pathStarted = false;

  // flags
  private boolean intakeOn = false;
  private boolean intakeSequenceRunning = false;
  private boolean shooting = false;

  // Flywheel/turret constants (copied)
  private static final double AUTO_SHOOTER_RPM = 90.0; // close-mode target
  private long shooterWaitStartMs = -1;
  private static final long SHOOTER_WAIT_TIMEOUT_MS = 4000L;

  public FarmodeAuto() {}

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    // Create follower and build the 3 needed paths
    follower = Constants.createFollower(hardwareMap);
    paths = new Paths(follower);

    // Set starting pose (63,8 heading 90deg)
    follower.setStartingPose(new Pose(63.0, 8.0, Math.toRadians(90)));

    // timers
    intakeSequenceTimer = new Timer();
    waitTimer = new Timer();
    holdTimer = new Timer();
    shootTimer = new Timer();

    // --- Hardware mapping (copied from ExperimentalPedroAuto) ---
    // Shooter & turret motors + IMU + subsystems
    try {
      shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
      turretMotor = hardwareMap.get(DcMotor.class, "turret");

      shooterMotor.setDirection(DcMotor.Direction.REVERSE);
      turretMotor.setDirection(DcMotor.Direction.FORWARD);

      shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    } catch (Exception e) {
      panelsTelemetry.debug("Init", "Failed to map shooter/turret motors: " + e.getMessage());
    }

    // IMU
    try {
      imu = hardwareMap.get(BNO055IMU.class, "imu");
      BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
      imuParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;
      imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
      imu.initialize(imuParams);
    } catch (Exception e) {
      panelsTelemetry.debug("Init", "IMU not found or failed to init: " + e.getMessage());
    }

    // Subsystems: Flywheel & TurretController (if available)
    try {
      if (shooterMotor != null) flywheel = new Flywheel(shooterMotor, telemetry);
      if (turretMotor != null) turretController = new TurretController(turretMotor, imu, telemetry);

      if (turretController != null) {
        turretController.captureReferences();
        turretController.resetPidState();
      }

      // Do NOT enable shooter in init (same as ExperimentalPedroAuto)
      if (flywheel != null) {
        flywheel.setModeFar(false); // close mode target, but do not enable shooter here
      }
    } catch (Exception e) {
      panelsTelemetry.debug("Init", "Flywheel/TurretController creation error: " + e.getMessage());
    }

    // --- Intake & compression hardware (same names as teleop) ---
    try {
      intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
      leftCompressionServo = hardwareMap.get(Servo.class, "leftCompressionServo");
      rightCompressionServo = hardwareMap.get(Servo.class, "rightCompressionServo");

      intakeMotor.setDirection(DcMotor.Direction.REVERSE);
      intakeMotor.setPower(0.0);
      leftCompressionServo.setPosition(LEFT_COMPRESSION_OFF);
      rightCompressionServo.setPosition(RIGHT_COMPRESSION_OFF);
    } catch (Exception e) {
      panelsTelemetry.debug("Init", "Intake/compression mapping failed: " + e.getMessage());
    }

    // Claw servo (mapped for completeness)
    try {
      clawServo = hardwareMap.get(Servo.class, "clawServo");
      clawServo.setPosition(0.63); // ensure open
    } catch (Exception e) {
      panelsTelemetry.debug("Init", "Claw servo mapping failed: " + e.getMessage());
    }

    panelsTelemetry.debug("Status", "Initialized FarmodeAuto (Experimental-style). Shooter remains OFF until start()");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void init_loop() {
    // warm up flywheel/turret readings like ExperimentalPedroAuto
    if (flywheel != null) {
      flywheel.update(System.currentTimeMillis(), false);
    }
    if (turretController != null) {
      turretController.update(false, 0.0);
    }
  }

  @Override
  public void start() {
    // Enable flywheel now and set close-range target RPM (same behavior as ExperimentalPedroAuto.start())
    if (flywheel != null) {
      flywheel.setShooterOn(true);
      flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
    }
    if (turretController != null) {
      turretController.captureReferences();
      turretController.resetPidState();
    }

    // We do not wait for shooter to reach target; start driving immediately per your requested flow.
    state = AutoState.TO_52;
    pathStarted = false;
  }

  @Override
  public void loop() {
    // Update Pedro follower
    follower.update();

    // Keep flywheel and turret updated while opmode runs (copied pattern)
    long nowMs = System.currentTimeMillis();
    if (flywheel != null) {
      flywheel.handleLeftTrigger(false);
      flywheel.update(nowMs, false);
    }
    if (turretController != null) {
      turretController.update(false, 0.0);
    }

    // Run the small FSM implementing your sequence
    switch (state) {
      case TO_52:
        // Start path from (63,8) -> (52,14)
        if (!pathStarted) {
          follower.followPath(paths.Path1);
          pathStarted = true;
          panelsTelemetry.debug("State", "TO_52: following Path1 -> (52,14)");
        }
        // arrival when follower is no longer busy
        if (pathStarted && !follower.isBusy()) {
          pathStarted = false;
          panelsTelemetry.debug("Arrived", "52,14 - starting intake sequence");
          // start intake (copy from ExperimentalPedroAuto)
          startIntake();
          // start intake-sequence timer
          intakeSequenceTimer.resetTimer();
          intakeSequenceRunning = true;
          state = AutoState.RUN_INTAKE_SEQ;
        }
        break;

      case RUN_INTAKE_SEQ:
        // run intake sequence (timed). When done, start 2s wait (intake stays ON)
        if (intakeSequenceRunning) {
          if (intakeSequenceTimer.getElapsedTimeSeconds() >= INTAKE_SEQUENCE_SECONDS) {
            intakeSequenceRunning = false;
            waitTimer.resetTimer();
            panelsTelemetry.debug("Action", "Intake sequence completed - starting 2s wait (intake stays ON)");
            state = AutoState.WAIT_AFTER_INTK;
          } else {
            panelsTelemetry.debug("IntakeSeq", String.format("running %.2fs/%.2fs",
                    intakeSequenceTimer.getElapsedTimeSeconds(), INTAKE_SEQUENCE_SECONDS));
          }
        }
        break;

      case WAIT_AFTER_INTK:
        // after 2s wait, drive to (16,14) while intake remains ON
        if (waitTimer.getElapsedTimeSeconds() >= WAIT_AFTER_INTAKE_SECONDS) {
          if (!pathStarted) {
            follower.followPath(paths.Path2);
            pathStarted = true;
            panelsTelemetry.debug("State", "WAIT_AFTER_INTK -> following Path2 -> (16,14) with intake ON");
          }
          state = AutoState.TO_16;
        } else {
          panelsTelemetry.debug("Waiting", String.format("%.2fs/%.2fs", waitTimer.getElapsedTimeSeconds(), WAIT_AFTER_INTAKE_SECONDS));
        }
        break;

      case TO_16:
        // driving to (16,14) while intake ON
        if (pathStarted && !follower.isBusy()) {
          pathStarted = false;
          panelsTelemetry.debug("Arrived", "16,14 - holding intake ON for 1s");
          holdTimer.resetTimer();
          state = AutoState.HOLD_AT_16;
        }
        break;

      case HOLD_AT_16:
        // keep intake ON for 1s, then drive back to 52,14
        if (holdTimer.getElapsedTimeSeconds() >= HOLD_AT_16_SECONDS) {
          if (!pathStarted) {
            follower.followPath(paths.Path3);
            pathStarted = true;
            panelsTelemetry.debug("State", "HOLD_AT_16 -> following Path3 -> (52,14)");
          }
          state = AutoState.TO_52_AGAIN;
        } else {
          panelsTelemetry.debug("HoldAt16", String.format("%.2fs/%.2fs", holdTimer.getElapsedTimeSeconds(), HOLD_AT_16_SECONDS));
        }
        break;

      case TO_52_AGAIN:
        if (pathStarted && !follower.isBusy()) {
          pathStarted = false;
          panelsTelemetry.debug("Arrived", "52,14 (final) - starting shooting (3)");
          // ensure flywheel is ON (should already be on), keep intake ON during shooting
          if (flywheel != null) {
            flywheel.setShooterOn(true);
            flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
          }
          shootTimer.resetTimer();
          shooting = true;
          state = AutoState.SHOOT;
        }
        break;

      case SHOOT:
        // during shooting, keep updating subsystems (done at top of loop). Wait SHOOT_SECONDS then stop intake.
        if (shootTimer.getElapsedTimeSeconds() >= SHOOT_SECONDS) {
          shooting = false;
          panelsTelemetry.debug("Action", "Shooting period complete - stopping intake");
          stopIntake(); // stop intake after shooting period
          // leave flywheel on or off as desired; ExperimentalPedroAuto turns flywheel off in stop()
          state = AutoState.FINISHED;
        } else {
          panelsTelemetry.debug("Shooting", String.format("%.2fs/%.2fs", shootTimer.getElapsedTimeSeconds(), SHOOT_SECONDS));
        }
        break;

      case FINISHED:
        // do nothing further; maintain telemetry/hardware until OpMode stop()
        break;

      case INIT:
      default:
        break;
    }

    // Telemetry (copied style)
    panelsTelemetry.debug("State", state.name());
    try {
      panelsTelemetry.debug("X", follower.getPose().getX());
      panelsTelemetry.debug("Y", follower.getPose().getY());
      panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    } catch (Exception ignored) {}
    if (flywheel != null) {
      panelsTelemetry.debug("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
      panelsTelemetry.debug("Fly Target", String.format("%.1f", flywheel.getTargetRPM()));
      panelsTelemetry.debug("Fly On", flywheel.isShooterOn());
      panelsTelemetry.debug("Fly AtTarget", flywheel.isAtTarget());
    }
    if (turretMotor != null && turretController != null) {
      panelsTelemetry.debug("Turret Enc", turretMotor.getCurrentPosition());
      panelsTelemetry.debug("Turret Power", turretController.getLastAppliedPower());
      panelsTelemetry.debug("TurretTrackingEnabled", String.valueOf(true));
    }
    if (intakeMotor != null) {
      panelsTelemetry.debug("Intake Power", intakeMotor.getPower());
      panelsTelemetry.debug("LeftCompPos", leftCompressionServo != null ? leftCompressionServo.getPosition() : -1);
      panelsTelemetry.debug("RightCompPos", rightCompressionServo != null ? rightCompressionServo.getPosition() : -1);
    }
    if (clawServo != null) {
      panelsTelemetry.debug("ClawPos", clawServo.getPosition());
    }
    panelsTelemetry.debug("IntakeOn", intakeOn);
    panelsTelemetry.debug("IntakeSeqRunning", intakeSequenceRunning);
    panelsTelemetry.debug("Shooting", shooting);
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void stop() {
    // turn off flywheel and turret safely (copied)
    if (flywheel != null) {
      flywheel.setShooterOn(false);
      flywheel.update(System.currentTimeMillis(), false);
    }
    if (turretController != null) {
      turretController.update(false, 0.0);
    }

    // Ensure intake off and claw open
    stopIntake();
    if (clawServo != null) clawServo.setPosition(0.63);
    state = AutoState.FINISHED;

    panelsTelemetry.debug("Status", "Stopped");
    panelsTelemetry.update(telemetry);
  }

  // --- Intake helpers (copied from ExperimentalPedroAuto) ---
  private void startIntake() {
    try {
      if (intakeMotor != null) intakeMotor.setPower(INTAKE_ON_POWER);
      if (leftCompressionServo != null) leftCompressionServo.setPosition(LEFT_COMPRESSION_ON);
      if (rightCompressionServo != null) rightCompressionServo.setPosition(RIGHT_COMPRESSION_ON);
      intakeOn = true;
    } catch (Exception e) {
      panelsTelemetry.debug("Intake", "startIntake error: " + e.getMessage());
    }
  }

  private void stopIntake() {
    try {
      if (intakeMotor != null) intakeMotor.setPower(0.0);
      if (leftCompressionServo != null) leftCompressionServo.setPosition(LEFT_COMPRESSION_OFF);
      if (rightCompressionServo != null) rightCompressionServo.setPosition(RIGHT_COMPRESSION_OFF);
      intakeOn = false;
    } catch (Exception e) {
      panelsTelemetry.debug("Intake", "stopIntake error: " + e.getMessage());
    }
  }

  // === Paths (3 paths only) ===
  public static class Paths {
    public PathChain Path1; // 63,8 -> 52,14
    public PathChain Path2; // 52,14 -> 16,14
    public PathChain Path3; // 16,14 -> 52,14

    public Paths(Follower follower) {
      Path1 = follower
              .pathBuilder()
              .addPath(new BezierLine(new Pose(63.000, 8.000), new Pose(52.000, 14.000)))
              .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(109))
              .build();

      Path2 = follower
              .pathBuilder()
              .addPath(new BezierLine(new Pose(52.000, 14.000), new Pose(16.000, 14.000)))
              .setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(180))
              .build();

      Path3 = follower
              .pathBuilder()
              .addPath(new BezierLine(new Pose(16.000, 14.000), new Pose(52.000, 14.000)))
              .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(109))
              .build();
    }
  }
}