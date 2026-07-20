package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.BallColor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class SpindexerUpd {
    //Slot enum objects
    public enum SLOT { Empty, Green, Purple, Unknown}
    //Robot subsystem
    public RobotHardware robot;
    //Variables
    public SLOT[] slots;
    public int currentPos; // This is the only variable we need to track
    public int prevPos;
    public int index;
    public double colorValue;
    public SLOT stableColor;
    //private final List<SLOT> voteBuffer = new ArrayList<>();
    // -------------------------------
    // NEW ! - Incremental servo stepping state
    // -------------------------------
    private double servoTargetPos = 0.0;      // absolute servo target (0..1)
    private boolean servoBusy = false;

    //-------------------------------------------------------------
    // --- Constructor ---
    //-------------------------------------------------------------
    public SpindexerUpd(RobotHardware robot, SLOT slot0, SLOT slot1, SLOT slot2, int startPos) {
        this.robot = robot;
        this.slots = new SLOT[]{slot0, slot1, slot2};
        this.currentPos = startPos;
        RuntoPosition(currentPos);
    }
    //-------------------------------------------------------------
    // --- MOVEMENT METHODS ---
    // Moves to a specific position. Uses Math.floorMod to ensure
    // the index is always 0, 1, or 2 regardless of how high n is.
    //-------------------------------------------------------------
    public void RuntoPosition(int n) {
        prevPos = currentPos;
        currentPos = n;
        // Logical safety: map n to 0, 1, or 2
        index = Math.floorMod(currentPos, 6);
        robot.spindexerServo.setPosition(spindexerPositions[index]);
    }

    public void RunToNext() {
        RuntoPosition(currentPos + 1);
    } // Run to next position
    public void setCurrentPos(int pos){ this.currentPos = pos;} // new to set current post from outside

    //==================================================
    // Update Servo Step when requestServoPosition() is called.
    // Update Servo Step - Incremental Servo Stepping
    //==================================================
    /** !!New!! method to run servo to specific position
     * - Use requestServoPosition() + updateServoStep() Togeter.
     * - updateServoSetp() method needs to be called every loop in shooter & intake
     * - Call updateServoStep() EVERY loop() (or every FSM update tick)
     * - Returns true when the servo has reached the target (within tolerance)
     * - check Servo busy method.
     * */
    public void requestServoPosition(double pos) {
        servoTargetPos = clamp01(pos);
        servoBusy = true;
    }
    public boolean updateServoStep() {
        if (!servoBusy) return true;
        double currentCmd = robot.spindexerServo.getPosition(); // last commanded, not actual
        double error = servoTargetPos - currentCmd;
        if (Math.abs(error) <= servoTolerance) {
            robot.spindexerServo.setPosition(servoTargetPos); // snap to exact target
            servoBusy = false;
            return true;
        }
        double step = Math.copySign(Math.min(Math.abs(error), servoStepSize), error);
        robot.spindexerServo.setPosition(clamp01(currentCmd + step));
        return false;
    }
    /// Check servo states
    public boolean isServoBusy() {
        return servoBusy;
    }
    //==================================================
    // --- COLOR Detection Methods (Continuous Count) ---
    //==================================================
    /** No calculated Slot needed. Just use currentPos % 3.
      * write the color directly to slot */
    public void writeToCurrentSlot(int slotnumber, BallColor color) {
        if (color == BallColor.GREEN){
            stableColor = SLOT.Green;}
        else if (color == BallColor.PURPLE){
            stableColor = SLOT.Purple;}
        else {stableColor = SLOT.Unknown;}
        // Apply result to the current logical index
        slots[slotnumber] = stableColor;
    }

    //==================================================
    // --- UTILITY METHODS ---
    //==================================================
    public int count(SLOT target) {
        int counter = 0;
        for (SLOT s : slots) if (s == target) counter++;
        return counter;
    }
    /// find next higher slot
    public double findNextHigherSlot(double currentPos, double[] slots) {
        double closest = -1;
        for (double slot : slots) {
            if (slot >= currentPos) {
                closest = slot;
                break; // first higher is the closest higher
            }
        }
        return closest; // -1 means none found
    }
    //==================================================
    // Getter and Setter Helper
    //==================================================
    public void resetSlot() {
        //for (int i = 0; i < slots.length; i++) slots[i] = SLOT.Empty;
        Arrays.fill(slots, SLOT.Empty);
    }
    public SLOT getCurrentSlotColor() {
        return slots[Math.floorMod(currentPos, 3)];
    }
    public double getServoPosition() {
        return robot.spindexerServo.getPosition();
    }
    //==================================================
    // Shooter End Helper
    //==================================================
    public void SpindexerShootingEnd() {
        requestServoPosition(spindexerPositions[0]);
    }
    //==================================================
    // Value Clamp Helper
    //==================================================
    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}