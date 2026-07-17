package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.BALL_PRESENT_THRESHOLD_MM;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.greenRangeHigh;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.greenRangeLow;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.purpleRangeHigh;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.purpleRangeLow;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SlotSensor {
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    public SlotSensor(HardwareMap hardwareMap, String deviceName){
        colorSensor=hardwareMap.get(ColorSensor.class,deviceName);
        distanceSensor=hardwareMap.get(DistanceSensor.class,deviceName);
    }

    public double getDistance(DistanceUnit unit){
        return distanceSensor.getDistance(unit);
    }

    public float[] getColourHSV(){
        float[] hsvValues=new float[3];
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues;
    }

    public boolean checkBall(){
        //no voting, may change in testing
        //float hue=getColourHSV()[0]; Commented out to save computation
        return (getDistance(DistanceUnit.MM) < BALL_PRESENT_THRESHOLD_MM);
        /*For colour detection:
                &&((greenRangeLow[0] < hue && hue < greenRangeLow[1])
                ||(greenRangeHigh[0] < hue && hue < greenRangeHigh[1])
                ||(purpleRangeLow[0] < hue && hue < purpleRangeLow[1])
                ||(purpleRangeHigh[0] < hue && hue < purpleRangeHigh[1]))
         */
    }
}
