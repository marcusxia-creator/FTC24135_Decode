package org.firstinspires.ftc.teamcode.TeleOps;

import android.graphics.Color;

import com.qualcomm.robotcore.robot.Robot;

public class ColorDetection {
    private RobotHardware robot;

    public ColorDetection(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Detect the current color based on HSV hue.
     * Returns "Purple", "Green", or "Unknown".
     */
    public String detectColorHue() {
        float hsv[] = new float[3];
        Color.RGBToHSV(
                robot.colorSensor.red() * 8,
                robot.colorSensor.green() * 8,
                robot.colorSensor.blue() * 8,
                hsv
        );
        float hue = hsv[0];

        if (hue >= 250 && hue <= 290) return "Purple";
        if (hue >= 80 && hue <= 160) return "Green";
        return "Unknown";
    }

    /**
     * Returns the raw hue value (useful for telemetry or tuning).
     */
    public float getHue() {
        float hsv[] = new float[3];
        Color.RGBToHSV(
                robot.colorSensor.red() * 8,
                robot.colorSensor.green() * 8,
                robot.colorSensor.blue() * 8,
                hsv
        );
        return hsv[0];
    }

    /**
     * Returns raw RGB readings for diagnostics.
     */
    public String getRawRGB() {
        return "R:" + robot.colorSensor.red() + " G:" + robot.colorSensor.green() + " B:" + robot.colorSensor.blue();
    }
}
