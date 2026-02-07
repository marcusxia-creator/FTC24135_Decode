package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import android.graphics.Color;


public class AutoColorDetection {
    private final RobotHardware robot;
    private AutoBallColors lastReadColor = AutoBallColors.UNKNOWN;
    private AutoBallColors currentColor;
    private AutoBallColors stableColor;
    private int stableCount = 0;
    private final int REQUIRED_STABLE_COUNT = 3; // number of consistent readings (~0.3s if called every 20ms)
    private final ElapsedTime timer = new ElapsedTime();

    private boolean isDetectionRunning = false;
    private static final int SENSOR_GAIN = 0;

    public AutoColorDetection(RobotHardware robot) {
        this.robot = robot;
    }

    public void detectInit(){
        this.isDetectionRunning = true;
        this.lastReadColor = AutoBallColors.UNKNOWN;
        this.stableColor = AutoBallColors.UNKNOWN;
        this.timer.reset();
    }


    /// color detection update!
    public void updateDetection(){
        if(isBallPresent()){
            ///convert Hue value to color enum name
            currentColor = AutoBallColors.fromHue(gethue());
            if (currentColor == lastReadColor && currentColor.isKnown())
            {
                stableCount++;
                if(stableCount > REQUIRED_STABLE_COUNT){
                    stableColor = currentColor;
                    timer.reset();
                }
                lastReadColor = currentColor;
            }else {
                stableCount = 0;
            }
        }
    }

    /// get color enum name
    public AutoBallColors getColor(){
        if (stableColor.isKnown()){
            return stableColor;
        }else{
            return AutoBallColors.UNKNOWN;
        }
    }

    /// Get Hue value from color sensor
    private float gethue(){
        return getHsvValues()[0];
    }

    /// Generate HSV values from RGB values from color sensor for color detection
    private float[] getHsvValues(){
        float[] hsvValues = new float[3];
        Color.RGBToHSV(
                robot.colorSensor.red()*SENSOR_GAIN,
                robot.colorSensor.blue()*SENSOR_GAIN,
                robot.colorSensor.green()*SENSOR_GAIN,
                hsvValues
        );
        return hsvValues;
    }

    public boolean isBallPresent(){
        double distance = robot.distanceSensor.getDistance(DistanceUnit.MM);
        return distance < BALL_PRESENT_THRESHOLD_MM;
    }


}