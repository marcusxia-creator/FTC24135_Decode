package org.firstinspires.ftc.teamcode.TeleOps;

import android.graphics.Color;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDetection {
    private RobotHardware robot;
    // For state tracking
    private BallColor lastColor = BallColor.UNKNOWN;
    private BallColor stableColor = BallColor.UNKNOWN;
    private int stableCount = 0;

    private ElapsedTime timer = new ElapsedTime();

    // Parameters
    private final int REQUIRED_STABLE_COUNT = 5; // number of consistent readings (~0.3s if called every 20ms)
    private final double TIMEOUT_S = 1.0;        // maximum time allowed to detect color
    public ColorDetection(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Detect the current color based on HSV hue.
     * Returns "Purple", "Green", or "Unknown".
     */
    public void startDetection() {
        stableColor = BallColor.UNKNOWN;
        lastColor = BallColor.UNKNOWN;
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

        BallColor currentColor = BallColor.fromHue(hue);

        /**Stability Check*/
        if (currentColor == lastColor && currentColor.isKnown())
        {
            stableCount++;
            if(stableCount > REQUIRED_STABLE_COUNT){
                stableColor = currentColor;
                timer.reset();
            }
        }else {
            stableCount = 0;
        }

        // --- Timeout reset ---
        if (timer.seconds() > TIMEOUT_S) {
            stableColor = BallColor.UNKNOWN;
            timer.reset();
        }

        lastColor = currentColor;
    }

    /** return is color stable boolean. */
    public boolean isColorStable(){
        return stableColor.isKnown();
    }

    /** check if the ball is present. */
    public boolean isBallPresent() {
        // Check if ball is in slot first
        double distance = robot.distanceSensor.getDistance(DistanceUnit.MM);
        return distance < RobotActionConfig.BALL_PRESENT_THRESHOLD_MM;
    }

    /** Returns the raw hue value (useful for telemetry or tuning). */
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

    /**Returns raw RGB readings for diagnostics.*/
    public String getRawRGB() {
        return "R:" + robot.colorSensor.red() + " G:" + robot.colorSensor.green() + " B:" + robot.colorSensor.blue();
    }

    public BallColor getStableColor(){ return stableColor;}
    public double getDistance(){ return robot.distanceSensor.getDistance(DistanceUnit.MM);}
}
