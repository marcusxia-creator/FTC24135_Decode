package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.distanceThreshold;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.greenRangeHigh;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.greenRangeLow;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.purpleRangeHigh;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.purpleRangeLow;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Spindexer {
    public enum SLOT{
        Empty,
        Green,
        Purple
    }

    public RobotHardware robot;
    public SLOT[] slots;
    public int currentSlot;
    //For jams
    public int prevSlot;

    Spindexer(RobotHardware robot, SLOT slot0,SLOT slot1,SLOT slot2, int currentSlot){
        this.robot = robot;
        slots = new SLOT[]{slot0, slot1, slot2};
        this.currentSlot = currentSlot;
        runToSlot(currentSlot);
    }

    public void writeToCurrent(SLOT a){
        slots[currentSlot]=a;
    }

    public void writeToCurrent(ColorSensor colorSensor, DistanceSensor distanceSensor) {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);

        if (distanceSensor.getDistance(DistanceUnit.MM)<distanceThreshold) {
            if ((greenRangeHigh[0] < hsvValues[0] && hsvValues[0] < greenRangeLow[1]) ||
                    greenRangeHigh[0] < hsvValues[0] && hsvValues[0] < greenRangeHigh[1]) {
                //Green
                writeToCurrent(Spindexer.SLOT.Green);
            } else if ((purpleRangeLow[0] < hsvValues[0] && hsvValues[0] < purpleRangeLow[1]) ||
                    purpleRangeHigh[0] < hsvValues[0] && hsvValues[0] < purpleRangeHigh[1]) {
                //Purple
                writeToCurrent(Spindexer.SLOT.Purple);
            }
        }
        else{
            writeToCurrent(SLOT.Empty);
        }
    }

    public void writeToCurrent(ColourCriterion[] criteria){
        //not yet implimented
    }

    public int count(SLOT a){
        int counter = 0;
        for(SLOT slot:slots){
            if(slot==a){
                counter++;
            }
        }
        return counter;
    }

    public Boolean checkFor(SLOT a){
        return count(a)>0;
    }

    public void runToSlot(){
        currentSlot = Math.floorMod(currentSlot,3);
        if(currentSlot==0){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot0);
        }
        if(currentSlot==1){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);
        }
        if(currentSlot==2){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot2);
        }
    }

    public void runToSlot(int n){
        prevSlot = currentSlot;
        currentSlot = n;
        runToSlot();
    }

    public Boolean runToSlot(SLOT a){
        if(checkFor(a)){
            int n=0;
            int distance = 4;

            //look for closest slot
            for(int i=0; i<=2; i++){
                if(slots[i]==a && Math.abs(i-currentSlot)<=distance){
                    distance=Math.abs(i-currentSlot);
                    n=i;
                }
            }
            runToSlot(n);
            return true;
        }
        else{
            return false;
        }
    }

    public void unJam(){
        runToSlot(prevSlot);
    }
}