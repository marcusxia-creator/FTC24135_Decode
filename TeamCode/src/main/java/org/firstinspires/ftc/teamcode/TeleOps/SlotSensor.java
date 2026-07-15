package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.BallColor;

public class SlotSensor {

    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    private static final int RGB_SCALE_FACTOR = 8;

    public enum SlotStatus {
        EMPTY,
        PURPLE,
        GREEN,
        UNKNOWN;

        public boolean hasObject(){
            return this !=EMPTY;
        }
        public boolean hasKnownBall(){
        return this == PURPLE || this ==GREEN;
        }
    }

    public SlotSensor(HardwareMap hardwareMap, String deviceName){
        colorSensor=hardwareMap.get(ColorSensor.class,deviceName);
        distanceSensor=hardwareMap.get(DistanceSensor.class,deviceName);
    }

    public double getDistance(DistanceUnit unit){
        return distanceSensor.getDistance(unit);
    }

    /**
     * Reads the RGB values and converts them to HSV.
     * The returned array contains:
     * index 0: hue
     * index 1: saturation
     * index 2: value
     * Do not modify the returned array because it is reused internally.
     * */
    public float[] getColourHSV(){
        float[] hsvValues=new float[3];
        Color.RGBToHSV(
                colorSensor.red() * RGB_SCALE_FACTOR,
                colorSensor.green() * RGB_SCALE_FACTOR,
                colorSensor.blue() * RGB_SCALE_FACTOR,
                hsvValues);
        return hsvValues;
    }
    public float getHue(){
        return getColourHSV()[0];
    }
    public float getSaturation(){
        return getColourHSV()[1];
    }
    public float getValue(){
        return getColourHSV()[2];
    }
    public String getRawRGB(){
        return "R:" + colorSensor.red() + " G:" + colorSensor.green() + " B:" + colorSensor.blue();
    }
    ///Checks only whether an object is close enough to occupy the slot.
    public boolean isBallPresent(){
        double distance=getDistance(DistanceUnit.MM);

        return Double.isFinite(distance) && distance>=0.0 && distance < BALL_PRESENT_THRESHOLD_MM;
    }

    /** * Checks whether the detected hue falls inside one of the accepted * green or purple hue ranges. */
    public boolean isTargetColour() {
        BallColor detectedColor = BallColor.fromHue(getHue());
        return detectedColor.isKnown();
    }

    /** * Returns true only when:
     * * 1. A ball is close enough to the distance sensor.
     * * 2. The detected color is within an accepted green or purple range. */
    public boolean ballCheck(){
        return isBallPresent() && isTargetColour();
    }


    public BallColor getBallColor(){
        if (!isBallPresent()) return BallColor.UNKNOWN;

        return BallColor.fromHue(getHue());
    }

    public SlotStatus getSlotStatus() {
        if (!isBallPresent()) {
            return SlotStatus.EMPTY;
        }

        switch (BallColor.fromHue(getHue())) {
            case PURPLE:
                return SlotStatus.PURPLE;

            case GREEN:
                return SlotStatus.GREEN;

            default:
                return SlotStatus.UNKNOWN;
        }
    }

    ///Helper - range check
    private boolean isWithinRange(double value, int[] range){
        if (range == null || range.length != 2) return false;
        return value > range[0] && value < range[1];
    }

}
