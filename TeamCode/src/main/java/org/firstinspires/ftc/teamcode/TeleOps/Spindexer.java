package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.distanceThreshold;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.greenRangeHigh;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.greenRangeLow;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.purpleRangeHigh;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.purpleRangeLow;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.spindexerZeroPos;

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
    public int currentPos;
    public int currentSlot;
    //For jams
    public int prevPos;

    Spindexer(RobotHardware robot, SLOT slot0,SLOT slot1,SLOT slot2, int currentPos){
        //Constructor
        this.robot = robot;
        slots = new SLOT[]{slot0, slot1, slot2};
        this.currentPos = currentPos;
        runToPos(currentPos);
    }

    public void calculateSlot(){
        if(currentPos==0) {
            currentSlot = -1;
        }
        else{
            currentSlot=Math.floorMod(currentPos-1,3);
        }
    }

    public void calculatePos(){
        currentPos=currentSlot+1;
    }

    /**
     * Saves slot value {@code a} into current slot
     * @param a The slot value to write into the current slot
     */
    public void writeToCurrent(SLOT a){
        calculateSlot();
        if(currentPos!=0) {
            slots[currentSlot] = a;
        }
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
        return slotColour(currentPos);
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
     * Counts the total number of spindexer slots that currently contain any of the inputed SLOTS
     */
    public int count(SLOT... a){
        int counter = 0;
        for(SLOT slot:a){
            counter+=count(slot);
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
     * @return {@code TRUE} if there is at least one instance of all given SLOT objects in the indexer, else {@code FALSE}
     */
    public Boolean checkFor(SLOT... slots){
        //checks
        for(SLOT slot:slots){
            if(!checkFor(slot)){
                return false;
            }
        }
        return true;
    }

    /**
     * Updates servo position to current slot, usually unused externally
     */
    public void runToPos(){
        currentPos = Math.floorMod(currentPos,3);
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerPositions[currentPos]);
    }

    /**
     * Reset slot position to 0
     */
    public void resetSlot() {
        slots[0] = SLOT.Empty;
        slots[1] = SLOT.Empty;
        slots[2] = SLOT.Empty;
    }
    /**
     * Runs to position {@code n} (0-5)
     */
    public void runToPos(int n){
        prevPos = currentPos;
        currentPos = n;
        calculateSlot();
        currentPos = Math.floorMod(currentPos,3);
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerPositions[currentPos]);
    }

    /**
     * Runs to Slot number {@code n} (0-2)
     */
    public void runToSlot(int n){
        prevPos = currentPos;
        currentSlot = n;
        calculatePos();
        runToPos();
    }

    /**
     * Runs spindexer to position before last movement
     */
    public void unJam(){
        runToPos(prevPos);
    }

    //Intaking Methods
    /**
     * Moves spindexer to position 1 slot 0, in preparation for intaking
     */

    /**
     * Movess spindexer forward one slot after intaking artifact
     * Note: Does not memorize, run WriteToCurrent before
     */
    public void IntakeNext(){
        runToPos(currentPos );
    }
    public void SpindexerBegin(int n){
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerPositions[n]);
    }
    public void RunToNext(){
        prevPos = currentPos;
        currentPos = currentPos+1;
        currentPos = Math.floorMod(currentPos,3);
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerPositions[currentPos]);
    }
    public void KickerRetract(){
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerPositions[0]);
    }
    public void SpindexerShootingEnd (){
        robot.spindexerServo.setPosition(spindexerZeroPos);
    }

    //Stop when currentPos==3 or count(SLOT.empty)==0

    //Shooting Methods
    /**
     * Moves spindexer to position 3 slot 2, in preparation for a simple sequential shoot. Usually does nothing
     */
    public void BeginSequShoot(){
        runToPos(currentPos-1);
    }

    /**
     * Moves spindexer to a slot where the motif can be sequentially shot. Run the same ShootNext function
     * @param motifGreen: the green artifact's index in the motif
     */
    public void BeginSortShoot(int motifGreen){
        int spindexerGreen=0;
        for(int i=0; i<=2; i++){
            if(slots[i]==SLOT.Green) {
                spindexerGreen=i;
                break;
            }
        }
        runToPos(3+Math.floorMod(motifGreen-spindexerGreen,3));
    }
    /**
     * Move's spindexer back one slot to shoot artifact
     * Note: Writes current slot to Empty
     * Does not control kicker
     */
    public void ShootNext(){
        writeToCurrent(SLOT.Empty);
        runToPos(currentPos-1);
    }
    //Stop when count(SLOT.empty)==3
}
