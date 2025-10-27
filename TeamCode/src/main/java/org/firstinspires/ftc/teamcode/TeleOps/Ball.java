package org.firstinspires.ftc.teamcode.TeleOps;

import androidx.annotation.NonNull;

public class Ball {
    public String ballColor;
    public int slotNumber;
    public double slotAngle;
    public boolean hasBall;

    //constructor ball
    public Ball(String ballColor, int slotPosition,double slotAngle, boolean hasBall){
        this.ballColor = ballColor;
        this.slotNumber = slotPosition;
        this.slotAngle = slotAngle;
        this.hasBall = hasBall;
    }

    //return and set ballColor
    public String getBallColor(){
        return ballColor;
    }
    public void setBallColor(String ballColor){
        this.ballColor = ballColor;
    }

    //return ball slotPosition
    public int getSlotPosition(){
        return slotNumber;
    }

    public double getSlotAngle(){
        return slotAngle;
    }

    //return and set hasBall
    public boolean hasBall() {return hasBall;}
    public void setHasBall(boolean hasBall) {
        this.hasBall = hasBall;
    }

    @NonNull
    public String toString(){
        return "Slot "+slotNumber + "("+slotAngle+")"+ballColor;
    }
}
