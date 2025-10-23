package org.firstinspires.ftc.teamcode.TeleOps;

import android.graphics.Color;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDetection {
    private RobotHardware robot;
    // For state tracking
    private String lastColor = "Unknown";
    private String stableColor = "Unknown";
    private int stableCount = 0;
    private boolean ballPresent = false;
    private ElapsedTime timer = new ElapsedTime();

    // Parameters
    private final int REQUIRED_STABLE_COUNT = 10; // number of consistent readings (~0.3s if called every 20ms)
    private final double TIMEOUT_MS = 500;        // maximum time allowed to detect color
    private final double BALL_PRESENT_THRESHOLD_MM = 10; // adjust per sensor mounting
    public ColorDetection(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Detect the current color based on HSV hue.
     * Returns "Purple", "Green", or "Unknown".
     */
    public void startDetection() {
        stableColor = "Unknown";
        lastColor = "Unknown";
        stableCount = 0;
        timer.reset();
    }

    /** call this loop inside color detection place*/
    public void updateDetection(){

        float[] hsv = new float[3];
        Color.RGBToHSV(
                robot.colorSensor.red() * 8,
                robot.colorSensor.green() * 8,
                robot.colorSensor.blue() * 8,
                hsv
        );
        float hue = hsv[0];

        String currentColor;

        if (hue >= 250 && hue <= 290) currentColor = "Purple";
        else if (hue >= 80 && hue <= 160) currentColor = "Green";
        else currentColor = "Unknown";

        //Stability Check

        if (currentColor.equals(lastColor) && !currentColor.equals("Unknow"))
        {
            stableCount++;
            if(stableCount >REQUIRED_STABLE_COUNT){
                stableColor = currentColor;
            }
        }else {
            stableCount = 0;
        }
        lastColor = currentColor;
    }

    public boolean isColorStable(){
        return !stableColor.equals("Unknow");
    }

    public boolean isBallPresent() {
        // Check if ball is in slot first
        double distance = robot.distanceSensor.getDistance(DistanceUnit.MM);
        return distance < BALL_PRESENT_THRESHOLD_MM;
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

    public String getStableColor(){ return stableColor;}
}
