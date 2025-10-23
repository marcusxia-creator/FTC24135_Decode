package org.firstinspires.ftc.teamcode.TeleOps;

import androidx.annotation.NonNull;

public class Ball {
    public String ballColor;
    public int slotPosition;

    public double slotAngle;

    public boolean hasBall;

    //constructor ball
    public Ball(String ballColor, int slotPosition,double slotAngle, boolean hasBall ){
        this.ballColor = ballColor;
        this.slotPosition = slotPosition;
        this.slotAngle = slotAngle;
        this.hasBall = hasBall;
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

    private boolean isBall(){
        return hasBall;
    }

    @NonNull
    public String toString(){
        return "Slot "+slotPosition + "("+slotAngle+")"+ballColor;
    }
}
