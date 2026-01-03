package org.firstinspires.ftc.teamcode.extras;

import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.GateController;

public class TelemetryHub {
    private final Telemetry tele;
    private final PanelsTelemetry panels;

    private double flyCurrent = 0.0;
    private double flyTarget = 0.0;
    private boolean gateClosed = false;
    private GateController.GateCycleState gateState = GateController.GateCycleState.IDLE;
    private double hoodL = 0.0, hoodR = 0.0;
    private double loopMs = 0.0;

    public TelemetryHub(Telemetry tele, PanelsTelemetry panels) {
        this.tele = tele;
        this.panels = panels;
    }

    public void setFlywheel(double current, double target) {
        this.flyCurrent = current;
        this.flyTarget = target;
    }


    public void setLoopMs(double loopMs) {
        this.loopMs = loopMs;
    }

    public void push() {
        tele.addData("Fly RPM", String.format("%.1f", flyCurrent));
        tele.addData("Fly Target", String.format("%.1f", flyTarget));


        tele.addData("Loop ms", String.format("%.2f", loopMs));
        tele.update();

        // Optional Panels telemetry
        // if (panels != null) {
        //     panels.debug("loop.ms", String.valueOf(loopMs));
        //     panels.update(tele);
        // }
    }
}