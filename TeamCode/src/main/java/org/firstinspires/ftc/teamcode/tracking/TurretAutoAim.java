package org.firstinspires.ftc.teamcode.tracking;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Simple field-centric turret auto-aim toward a fixed field goal.
 * Computes heading error (goal bearing minus robot heading) and drives turret with P control.
 */
public class TurretAutoAim {

    private final DcMotor turretMotor;
    private final Pose goalPose;
    private double kP = 0.6;       // tune as needed
    private double maxPower = 0.5; // clamp power for safety

    public TurretAutoAim(DcMotor turretMotor, Pose goalPose) {
        this.turretMotor = turretMotor;
        this.goalPose = goalPose;
    }

    public void setGains(double kP, double maxPower) {
        this.kP = kP;
        this.maxPower = Math.abs(maxPower);
    }

    public void update(Pose robotPose) {
        if (robotPose == null) {
            turretMotor.setPower(0.0);
            return;
        }
        double targetHeading = Math.atan2(goalPose.getY() - robotPose.getY(),
                                          goalPose.getX() - robotPose.getX());
        double robotHeading = robotPose.getHeading();
        double error = angleWrap(targetHeading - robotHeading);
        double power = kP * error;
        power = Math.max(-maxPower, Math.min(maxPower, power));
        turretMotor.setPower(power);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
