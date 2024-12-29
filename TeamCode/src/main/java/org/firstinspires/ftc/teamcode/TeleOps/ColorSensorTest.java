package org.firstinspires.ftc.teamcode.TeleOps;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp (name= "Color_Sensor_Test", group = "org/firstinspires/ftc/teamcode/OpMode")
public class ColorSensorTest extends OpMode {

    public ColorSensor colorSensor;

    float hsvValues[] = {0F,0F,0F};

    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    public void loop() {

        if ((colorSensor.argb()) > 340 || colorSensor.argb() < 30) {
            telemetry.addLine("Color Red");
        }
        else if (colorSensor.green() == colorSensor.blue()) {
            telemetry.addLine("Color Blue");
        }
        else {
            telemetry.addLine("No Color Detected");
        }

        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
    }
}
