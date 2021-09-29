package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;

public class dashboard implements subsystem {

    ElapsedTime dashboardTimer = new ElapsedTime();

    public static TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init(HardwareMap hwmap) {
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    @Override
    public void initNoReset(HardwareMap hwmap) {
        init(hwmap);
    }

    @Override
    public void update() {

        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        dashboardTimer.reset();

    }

    @Override
    public Vector3D subsystemState() {
        return null;
    }


}
