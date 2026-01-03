package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RumbleHelper {
    public static void rumble(Gamepad gp, int ms) {
        if (gp == null) return;
        try { gp.rumble(ms); } catch (Throwable ignored) {}
    }

    public static void rumbleBoth(Gamepad gp1, Gamepad gp2, int ms) {
        rumble(gp1, ms);
        rumble(gp2, ms);
    }
}