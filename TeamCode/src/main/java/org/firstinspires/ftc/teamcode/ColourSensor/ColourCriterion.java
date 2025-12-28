package org.firstinspires.ftc.teamcode.ColourSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColourCriterion{
    Range[] ranges = {};

    public Boolean met;
    public ElapsedTime timer;

    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    ColourCriterion(ColorSensor colorSensor, DistanceSensor distanceSensor, Range... ranges){
        this.colorSensor=colorSensor;
        this.distanceSensor=distanceSensor;

        this.ranges=ranges;

        timer = new ElapsedTime();
        met=false;
    }

    boolean check(){
        met=true;
        for(Range range : ranges){
            if(!range.check(colorSensor,distanceSensor)){
                met=false;
                timer.reset();
                break;
            }
        }
        return met;
    }
}

class Range{
    enum RangeType{
        hue,
        value,
        prox
    }

    RangeType type;

    double lower;
    double higher;

    Range(RangeType type, double lower, double higher){
        this.type=type;
        this.lower=lower;
        this.higher=higher;
    }

    boolean check(ColorSensor colorSensor, DistanceSensor distanceSensor){
        double x=0;

        float[] hsv=new float[3];
        Color.RGBToHSV(colorSensor.red(),colorSensor.green(), colorSensor.blue(), hsv);

        switch(type){
            case hue:
                x = hsv[0];
                break;
            case value:
                x = hsv[2];
                break;
            case prox:
                x = distanceSensor.getDistance(DistanceUnit.CM);
                break;
        }

        return lower<x && x<higher;
    }
}