package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

/**
 * A single calibration sample: a field pose and the RPM that
 * scores reliably from that pose.
 *
 * Only the (x, y) of the pose is used for distance computation;
 * heading is stored for reference but does not affect the regression.
 */
public class CalibrationPoint {

    public final Pose  pose;
    public final double rpm;

    public CalibrationPoint(Pose pose, double rpm) {
        this.pose = pose;
        this.rpm  = rpm;
    }
}