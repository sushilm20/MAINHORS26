package org.firstinspires.ftc.teamcode.autopaths;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Implements:
 * - Start at (63,8) heading 90deg
 * - Path1 -> (52,14)
 *   - Run intake sequence once (placeholder for ExperimentalHors code)
 *   - After intake sequence completes, wait 2 seconds (intake stays ON)
 * - Path2 (while intake ON) -> (16,14)
 *   - Once arrived, keep intake ON for 1 second
 * - Path3 -> (52,14)
 *   - Shoot 3 (placeholder for ExperimentalHors shooting code), intake ON during shooting
 * - End at (52,14)
 *
 * NOTE: Replace the placeholder methods (startIntake, stopIntake, runIntakeSequence,
 * shootThree) with the actual implementations from your ExperimentalHors class.
 */
@Autonomous(name = "Farmode Auto (sequence)", group = "Autonomous")
@Configurable
public class FarmodeAuto extends OpMode {

  private TelemetryManager panelsTelemetry;
  public Follower follower;
  private Paths paths;

  private ElapsedTime runtime = new ElapsedTime();

  // State machine states
  // 1 = drive to 52,14 (Path1)
  // 2 = at 52,14: run intake sequence, then wait 2s; after that -> drive to 16,14 (Path2)
  // 3 = driving to 16,14
  // 4 = at 16,14: keep intake on for 1s then -> drive to 52,14 (Path3)
  // 5 = driving to 52,14
  // 6 = at 52,14: shoot 3, finish
  private int state = 1;

  // Intake / sequence timers / flags
  private boolean intakeOn = false;
  private boolean intakeSequenceRunning = false;
  private double intakeSequenceDuration = 1500.0; // ms, placeholder for actual sequence length
  private long intakeSequenceStart = 0L;

  private long waitStart = 0L;
  private double waitDuration = 2000.0; // 2 seconds wait after intake sequence

  private long at16HoldStart = 0L;
  private double at16HoldDuration = 1000.0; // 1 second hold at 16,14

  private boolean shooting = false;
  private boolean shotDone = false;
  private double shootDuration = 1500.0; // ms placeholder
  private long shootStart = 0L;

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(63.0, 8.0, Math.toRadians(90)));

    paths = new Paths(follower);

    panelsTelemetry.debug("Status", "Initialized FarmodeAuto (sequence)");
    panelsTelemetry.update(telemetry);
    runtime.reset();
  }

  @Override
  public void loop() {
    follower.update();

    switch (state) {
      case 1:
        // Drive to (52,14)
        if (!follower.isFollowingPath()) {
          follower.follow(paths.Path1);
          panelsTelemetry.debug("State", "1 - Starting Path1 -> (52,14)");
        }
        if (follower.isPathFinished()) {
          // arrived at shoot pose (52,14)
          panelsTelemetry.debug("Arrived", "52,14 - starting intake sequence");
          startIntake(); // intake ON during sequences and movements per user's request
          startIntakeSequence();
          state = 2;
        }
        break;

      case 2:
        // At 52,14: run intake sequence; when intake sequence completes -> wait 2s -> drive to 16,14
        if (intakeSequenceRunning) {
          if (isIntakeSequenceDone()) {
            intakeSequenceRunning = false;
            waitStart = System.currentTimeMillis();
            panelsTelemetry.debug("Action", "Intake sequence complete. Starting 2s wait.");
          }
        } else {
          // waiting after sequence
          long elapsed = System.currentTimeMillis() - waitStart;
          panelsTelemetry.debug("Waiting", elapsed + "ms / " + (long) waitDuration + "ms");
          if (elapsed >= (long) waitDuration) {
            // begin driving to 16,14, intake remains ON
            if (!follower.isFollowingPath()) {
              follower.follow(paths.Path2);
              panelsTelemetry.debug("State", "2 -> starting Path2 -> (16,14) with intake ON");
            }
            state = 3;
          }
        }
        break;

      case 3:
        // Driving to (16,14) while intake stays on
        if (follower.isPathFinished()) {
          panelsTelemetry.debug("Arrived", "16,14 - hold intake for 1s");
          at16HoldStart = System.currentTimeMillis();
          state = 4;
        }
        break;

      case 4:
        // Keep intake on for 1 second at 16,14 then go to 52,14
        long held = System.currentTimeMillis() - at16HoldStart;
        panelsTelemetry.debug("HoldingAt16", held + "ms / " + (long) at16HoldDuration + "ms");
        if (held >= (long) at16HoldDuration) {
          // drive back to 52,14 (end)
          if (!follower.isFollowingPath()) {
            follower.follow(paths.Path3);
            panelsTelemetry.debug("State", "4 -> starting Path3 -> (52,14)");
          }
          state = 5;
        }
        break;

      case 5:
        // Drive to final (52,14)
        if (follower.isPathFinished()) {
          panelsTelemetry.debug("Arrived", "52,14 - start shooting 3 (intake ON)");
          startShootingThree();
          state = 6;
        }
        break;

      case 6:
        // Shooting sequence
        if (shooting) {
          if (isShootingDone()) {
            shooting = false;
            shotDone = true;
            panelsTelemetry.debug("Action", "Shooting complete. Stopping intake.");
            stopIntake();
          }
        }
        // finished: we can stay here
        break;

      default:
        break;
    }

    // Telemetry always
    panelsTelemetry.debug("State", state);
    panelsTelemetry.debug("PoseX", follower.getPose().getX());
    panelsTelemetry.debug("PoseY", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.debug("IntakeOn", intakeOn);
    panelsTelemetry.debug("IntakeSeqRunning", intakeSequenceRunning);
    panelsTelemetry.debug("Shooting", shooting);
    panelsTelemetry.update(telemetry);
  }

  // === Path definitions ===
  public static class Paths {
    public PathChain Path1; // 63,8 -> 52,14
    public PathChain Path2; // 52,14 -> 16,14
    public PathChain Path3; // 16,14 -> 52,14 (end)

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

  // ===== Placeholders for robot-specific actions =====
  // Replace these with the actual code from ExperimentalHors class.
  private void startIntake() {
    if (!intakeOn) {
      intakeOn = true;
      // TODO: call ExperimentalHors.setIntakePower(true) or similar
      panelsTelemetry.debug("Hardware", "startIntake() called (placeholder)");
    }
  }

  private void stopIntake() {
    if (intakeOn) {
      intakeOn = false;
      // TODO: call ExperimentalHors.setIntakePower(false) or similar
      panelsTelemetry.debug("Hardware", "stopIntake() called (placeholder)");
    }
  }

  private void startIntakeSequence() {
    intakeSequenceRunning = true;
    intakeSequenceStart = System.currentTimeMillis();
    // TODO: copy/paste ExperimentalHors intake sequence here (timers, motor commands, servos)
    panelsTelemetry.debug("Hardware", "startIntakeSequence() started (placeholder)");
  }

  private boolean isIntakeSequenceDone() {
    // Placeholder condition: run for intakeSequenceDuration ms. Replace with actual completion logic.
    if (!intakeSequenceRunning) return true;
    long elapsed = System.currentTimeMillis() - intakeSequenceStart;
    return elapsed >= (long) intakeSequenceDuration;
  }

  private void startShootingThree() {
    if (!shooting && !shotDone) {
      shooting = true;
      shootStart = System.currentTimeMillis();
      // TODO: call ExperimentalHors.shootThreeRings() or sequence to spin shooter, index 3 rings, etc.
      panelsTelemetry.debug("Hardware", "startShootingThree() started (placeholder)");
    }
  }

  private boolean isShootingDone() {
    // Placeholder: shooting takes shootDuration ms. Replace with the real shooting completion logic.
    long elapsed = System.currentTimeMillis() - shootStart;
    return elapsed >= (long) shootDuration;
  }
}
