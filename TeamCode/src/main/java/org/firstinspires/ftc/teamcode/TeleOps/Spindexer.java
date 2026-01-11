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
        //Constructor
        this.robot = robot;
        slots = new SLOT[]{slot0, slot1, slot2};
        this.currentSlot = currentSlot;
        runToSlot(currentSlot);
    }
    /**
     * Saves slot value {@code a} into current slot
     * @param a The slot value to write into the current slot
     */
    public void writeToCurrent(SLOT a){
        slots[currentSlot]=a;
    }

    /**
     * Processes colour sensor data, and saves data to current slot (including an empty value)
     * @param colorSensor The robot's colour sensor object
     * @param distanceSensor The robot's distance sensor object
     */
    public void writeToCurrent(ColorSensor colorSensor, DistanceSensor distanceSensor) {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);

        if (distanceSensor.getDistance(DistanceUnit.MM)<distanceThreshold) {
            if ((greenRangeLow[0] < hsvValues[0] && hsvValues[0] < greenRangeLow[1]) ||
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

    /**
     * Returns the colour of a given slot {@code n}
     * @return spindexer SLOT object
     */
    public SLOT slotColour(int n){
        return slots[n];
    }

    /**
     * Returns the colour of the current slot
     * @return spindexer SLOT object in current slot
     */
    public SLOT slotColour(){
        return slotColour(currentSlot);
    }

    /**
     * Counts the instances of SLOT {@code a} currently recorded in the spindexer
     */
    public int count(SLOT a){
        int counter = 0;
        for(SLOT slot:slots){
            if(slot==a){
                counter++;
            }
        }
        return counter;
    }

    /**
     * @return {@code TRUE} if there is at least one instance of the given SLOT object {@code a} in the indexer, else {@code FALSE}
     */
    public Boolean checkFor(SLOT a){
        //checks
        return count(a)>0;
    }

    /**
     * Updates servo position to current slot, usually unused=
     */
    public void runToSlot(){
        currentSlot = Math.floorMod(currentSlot, 3);
        if(currentSlot==0){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot0);
            robot.rightSpindexerServo.setPosition(RobotActionConfig.spindexerSlot0);
        }
        if(currentSlot==1){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);
            robot.rightSpindexerServo.setPosition(RobotActionConfig.spindexerSlot0);
        }
        if(currentSlot==2){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot2);
            robot.rightSpindexerServo.setPosition(RobotActionConfig.spindexerSlot0);
        }
    }

    /**
     * Runs to slot number {@code n} (0, 1, or 2)
     */
    public void runToSlot(int n){
        prevSlot = currentSlot;
        currentSlot = n;
        runToSlot();
    }

    /**
     * Runs to closest SLOT {@code a}, perfers the one on the right if both slots equal {@code a}
     * @return (@code FALSE} if no instances of SLOT {@code a} are found in the spindexer
     */
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

    public void sequenceShoot (){
        if (currentSlot > 0) {
            runToSlot(Math.floorMod(currentSlot - 1, 3));
        }
    }

    /**
     * A storage object to record a Motif, with a few methods
     */
    public static class Motif{
        //Motif constants
        /**
         * Constant green-purple-purple motif
         */
        public static Motif GPP = new Motif("GPP");
        /**
         * Constant purple-green-purple motif
         */
        public static Motif PGP = new Motif("PGP");
        /**
         * Constant purple-purple-green motif
         */
        public static Motif PPG = new Motif("PPG");

        public SLOT[] slots;
        public String name;


        public Motif(String name){
            this.name=name;
            slots = new SLOT[3];
            for(int i=0; i<3; i++){
                if(this.name.charAt(i)=='G'){
                    slots[i]=SLOT.Green;
                }
                else{
                    slots[i]=SLOT.Purple;
                }
            }
            name="";
            for(SLOT slot:slots){
                if(slot==SLOT.Green){
                    name+="G";
                }
                else{
                    name+="P";
                }
            }
        }
    }
}
