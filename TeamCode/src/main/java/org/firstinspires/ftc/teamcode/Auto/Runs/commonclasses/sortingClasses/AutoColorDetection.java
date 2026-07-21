package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import android.graphics.Color;


public class AutoColorDetection {
    private final RobotHardware robot;
    private final ElapsedTime timer = new ElapsedTime();

    private AutoBallColors[] slotColors = {
            AutoBallColors.UNKNOWN,
            AutoBallColors.UNKNOWN,
            AutoBallColors.UNKNOWN
    };

    public AutoColorDetection(RobotHardware robot) {
        this.robot = robot;
    }

    public void detectInit(){
        this.timer.reset();
    }

    /// Get Hue value from color sensor
    private float gethue(int slotNumber){
        return getHsvValues(slotNumber)[0];
    }

    /// Generate HSV values from RGB values from color sensor for color detection
    private float[] getHsvValues(int slotNumber){
        return robot.slotSensors.get(slotNumber).getColourHSV();
    }

    public boolean isBallPresent(int slotNumber){
        double distance = robot.slotSensors.get(slotNumber).getDistance(DistanceUnit.MM);
        return distance < BALL_PRESENT_THRESHOLD_MM;
    }

    public void setSlotColor(int slotNumber) {
        if (isBallPresent(slotNumber)) {
            slotColors[slotNumber] = AutoBallColors.fromHue(gethue(slotNumber));
        } else {
            slotColors[slotNumber] = AutoBallColors.UNKNOWN;
        }
    }

    public void updateSlotColors() {
        for (int slotNumber = 0; slotNumber < 3; slotNumber++) {
            setSlotColor(slotNumber);
        }
    }

    public AutoBallColors getSlotColor(int slotNumber) {
        return slotColors[slotNumber];
    }

    public boolean isSpindexerFull() {
        int fullSlots = 0;
        int fullSpindexer = 3;

        for (int slotNumber = 0; slotNumber < fullSpindexer; slotNumber++) {
            if (isBallPresent(slotNumber)) {
                fullSlots++;
            }
        }

        return fullSlots >= fullSpindexer;
    }


    public int findGreenSlot() {
        updateSlotColors();
        for (int slotNumber = 0; slotNumber < 3; slotNumber++) {
            if (slotColors[slotNumber] == AutoBallColors.GREEN) {
                return slotNumber;
            }
        }
        return -1;
    }
}