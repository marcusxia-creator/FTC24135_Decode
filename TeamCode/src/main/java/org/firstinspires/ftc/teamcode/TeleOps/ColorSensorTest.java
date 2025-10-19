package org.firstinspires.ftc.teamcode.TeleOps;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp (name= "Color_Sensor_Test", group = "org/firstinspires/ftc/teamcode/OpMode")
public class ColorSensorTest extends OpMode {

    /**
     * Color Range:
     * None: 106 - 110
     * Purple: 115 - 125, 200 - 230
     * Green: 120 - 130, 145 - 160
     */

    public ColorSensor colorSensor;
    public FtcDashboard dashboard;

    float[] hsvValues = {0F,0F,0F};

    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    public void loop() {

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        /*
        if ((colorSensor.argb()) > 340 || colorSensor.argb() < 30) {
            telemetry.addLine("Color Red");
        }
        else if (colorSensor.green() == colorSensor.blue()) {
            telemetry.addLine("Color Blue");
        }
        else {
            telemetry.addLine("No Color Detected");
        }
         */

        if (hsvValues[1] < 0.5) {
            telemetry.addLine("No object detected");
        }
        if (hsvValues[1] > 0.5 && hsvValues[1] < 0.67) {
            telemetry.addLine("The artifact is not direct above the color sensor");
        }
        if (hsvValues[1] > 0.7) {
            telemetry.addLine("The artifact is directly above the color sensor");
        }


        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);

        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());

        telemetry.addData("argb value", colorSensor.argb());
        telemetry.update();
    }
}
