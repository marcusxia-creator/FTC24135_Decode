package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.BALL_PRESENT_THRESHOLD_MM;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.greenRangeHigh;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.greenRangeLow;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.purpleRangeHigh;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.purpleRangeLow;
//import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.spindexerZeroPos;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SpindexerSimp {
    public enum SLOT { Empty, Green, Purple, Unknown }

    public RobotHardware robot;
    public SLOT[] slots;
    public int currentPos; // This is the only variable we need to track
    public int prevPos;
    public double colorValue;

    private final List<SLOT> voteBuffer = new ArrayList<>();

    // -------------------------------
    // NEW ! - Incremental servo stepping state
    // -------------------------------
    private double servoTargetPos = 0.0;      // absolute servo target (0..1)
    private boolean servoBusy = false;

    // TODO Tune these
    //public double servoStepSize;     // per update() call (try 0.05)
    //private double servoTolerance;    // "close enough" to finish 0.025

    public SpindexerSimp(RobotHardware robot, SLOT slot0, SLOT slot1, SLOT slot2, int startPos) {
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
    public void RuntoPositionOLD(int n) {
        prevPos = currentPos;
        currentPos = n;

        // Logical safety: map n to 0, 1, or 2
        int index = Math.floorMod(currentPos, 3);
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerPositions[index]);
    }

    public void RunToNext() {
        RuntoPosition(currentPos + 1);
    }

    public void unJam() {
        RuntoPosition(prevPos);
    }

    ///  New!! method to run servo to incremental position
    public void RuntoPosition(int n) {
        prevPos = currentPos;
        currentPos = n;

        // Logical safety: map n to 0, 1, or 2
        int index = Math.floorMod(currentPos, 3);
        servoTargetPos = RobotActionConfig.spindexerPositions[index];
        servoBusy = true;
    }

    ///  New!! method to run servo to specific position
    public void requestServoPosition(double pos) {
        servoTargetPos = clamp01(pos);
        servoBusy = true;
    }

    //==================================================
    // Update Servo Step
    //==================================================
    /**
     * Call this EVERY loop() (or every FSM update tick).
     * Returns true when the servo has reached the target (within tolerance).
     */
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

    public boolean isServoBusy() {
        return servoBusy;
    }

    //==================================================
    // --- COLOR SENSING (MAJORITY VOTE) ---
    //==================================================
    public void clearVoteBuffer() {
        voteBuffer.clear();
    }

    public void addVoteSample(ColorSensor colorSensor, DistanceSensor distanceSensor) {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        colorValue = hsvValues[0];

        if (distanceSensor.getDistance(DistanceUnit.MM) < BALL_PRESENT_THRESHOLD_MM) {
            if ((greenRangeLow[0] < hsvValues[0] && hsvValues[0] < greenRangeLow[1]) ||
                    (greenRangeHigh[0] < hsvValues[0] && hsvValues[0] < greenRangeHigh[1])) {
                voteBuffer.add(SLOT.Green);
            } else if ((purpleRangeLow[0] < hsvValues[0] && hsvValues[0] < purpleRangeLow[1]) ||
                    (purpleRangeHigh[0] < hsvValues[0] && hsvValues[0] < purpleRangeHigh[1])) {
                voteBuffer.add(SLOT.Purple);
            } else {
                voteBuffer.add(SLOT.Unknown);
            }
        } else {
            voteBuffer.add(SLOT.Empty);
        }
    }

    /**
     * Simplification: No calculateSlot needed. We just use currentPos % 3.
     */
    public void finalizeCurrentSlot() {
        if (voteBuffer.isEmpty()) return;
        int greenVotes = Collections.frequency(voteBuffer, SLOT.Green);
        int purpleVotes = Collections.frequency(voteBuffer, SLOT.Purple);
        int unKonwnVotes = Collections.frequency(voteBuffer, SLOT.Unknown);
        int emptyVotes = Collections.frequency(voteBuffer, SLOT.Empty);

        SLOT winner;
        if (greenVotes > purpleVotes && greenVotes > unKonwnVotes && greenVotes > emptyVotes) winner = SLOT.Green;
        else if (purpleVotes > greenVotes && purpleVotes > unKonwnVotes && purpleVotes > emptyVotes) winner = SLOT.Purple;
        else if (unKonwnVotes > greenVotes && unKonwnVotes > purpleVotes && unKonwnVotes > emptyVotes) winner = SLOT.Unknown;
        else winner = SLOT.Empty;

        // Apply result to the current logical index
        slots[Math.floorMod(currentPos, 3)] = winner;
    }

    //==================================================
    // --- UTILITY METHODS ---
    //==================================================
    public int count(SLOT target) {
        int counter = 0;
        for (SLOT s : slots) if (s == target) counter++;
        return counter;
    }

    // find next higher slot
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
        for (int i = 0; i < slots.length; i++) slots[i] = SLOT.Empty;
    }
    public SLOT getCurrentSlotColor() {
        return slots[Math.floorMod(currentPos, 3)];
    }

    //==================================================
    // Shooter End Helper
    //==================================================
    public void SpindexerShootingEnd() {
        robot.spindexerServo.setPosition(spindexerZeroPos);
    }

    //==================================================
    // Value Clamp Helper
    //==================================================
    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}