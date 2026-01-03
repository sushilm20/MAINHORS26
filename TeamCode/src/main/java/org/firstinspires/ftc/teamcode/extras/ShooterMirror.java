package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ShooterMirror {
    private final DcMotor primary;
    private final DcMotor secondary;

    public ShooterMirror(DcMotor primary, DcMotor secondary) {
        this.primary = primary;
        this.secondary = secondary;
    }

    public void update() {
        if (primary == null || secondary == null) return;
        secondary.setPower(primary.getPower());
    }
}