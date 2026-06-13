
package org.firstinspires.ftc.teamcode.TeleOps.Sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class ColorDetection {
    private RobotHardware robot;
    // For state tracking
    private BallColor lastColor = BallColor.UNKNOWN;
    private BallColor stableColor = BallColor.UNKNOWN;
    private int stableCount = 0;

    private ElapsedTime timer = new ElapsedTime();

    // Parameters
    private final int REQUIRED_STABLE_COUNT = 5; // number of consistent readings (~0.1s if called every 20ms)
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
                robot.frontColorSensor.red() * 8,
                robot.frontColorSensor.green() * 8,
                robot.frontColorSensor.blue() * 8,
                hsv
        );
        float hue = hsv[0];
        float value = hsv[2];

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
        double distance = robot.frontDistanceSensor.getDistance(DistanceUnit.MM);
        return distance < RobotActionConfig.BALL_PRESENT_THRESHOLD_MM;
    }

    /** Returns the raw hue value (useful for telemetry or tuning). */
    public float getHue(int sensor) {
        float hsv[] = new float[3];
        if (sensor == 0) {
            Color.RGBToHSV(
                    robot.frontColorSensor.red() * 8,
                    robot.frontColorSensor.green() * 8,
                    robot.frontColorSensor.blue() * 8,
                    hsv);
        } if (sensor == 1) {
            Color.RGBToHSV(
                    robot.rightColorSensor.red() * 8,
                    robot.rightColorSensor.green() * 8,
                    robot.rightColorSensor.blue() * 8,
                    hsv);
        } else {
            Color.RGBToHSV(
                    robot.leftColorSensor.red() * 8,
                    robot.leftColorSensor.green() * 8,
                    robot.leftColorSensor.blue() * 8,
                    hsv);
        }
        return hsv[0];
    }

    /**Returns raw RGB readings for diagnostics.*/
    public String getRawRGB() {
        return "R:" + robot.frontColorSensor.red() + " G:" + robot.frontColorSensor.green() + " B:" + robot.frontColorSensor.blue();
    }

    public BallColor getStableColor(){ return stableColor;}

    public double getDistance(){ return robot.frontDistanceSensor.getDistance(DistanceUnit.MM);}
}
