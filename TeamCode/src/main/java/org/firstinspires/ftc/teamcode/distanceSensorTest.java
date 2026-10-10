package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class distanceSensorTest extends OpMode {
    DigitalChannel distanceSensor;

    @Override
    public void init() {
        distanceSensor=hardwareMap.get(DigitalChannel.class,"distanceSensor");
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        telemetry.addData("State",distanceSensor.getState()?1:0);
        telemetry.update();
    }
}
