package org.firstinspires.ftc.teamcode.TeleOps;

import androidx.annotation.NonNull;

public class Ball {
    public String ballColor;
    public int slotPosition;

    public double slotAngle;

    public boolean release;

    //constructor ball
    public Ball(String ballColor, int slotPosition,double slotAngle, boolean release ){
        this.ballColor = ballColor;
        this.slotPosition = slotPosition;
        this.slotAngle = slotAngle;
        this.release = release;
    }

    //return ballColor
    private String getBallColor(){
        return ballColor;
    }

    //return ball slotPosition
    private int getSlotPosition(){
        return slotPosition;
    }

    private double getSlotAngle(){
        return slotAngle;
    }

    private boolean isRelease(){
        return release;
    }

    @NonNull
    public String toString(){
        return "Slot "+slotPosition + "("+slotAngle+")"+ballColor;
    }
}
