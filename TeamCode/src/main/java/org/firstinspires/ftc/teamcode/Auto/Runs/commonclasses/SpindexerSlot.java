package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

public class SpindexerSlot {

    public int slotPosition;
    public double slotAngle;
    public boolean hasBall;
    public BallColors slotColor;

    public SpindexerSlot (int slotPosition, double slotAngle){
        this.slotPosition = slotPosition;
        this.slotAngle = slotAngle;
        this.hasBall = false;
        this.slotColor = BallColors.UNKNOWN;
    }

    ///Set Methods
    public void setBall(boolean hasBall){
        this.hasBall = hasBall;
    }
    public void setSlotColor(BallColors slotColor){
        this.slotColor = slotColor;
    }

    ///Get Methods
    public boolean getHasBall() {return hasBall;}
    public double getSlotAngle() {return slotAngle;}
    public int getSlotPosition() {return slotPosition;}
    public BallColors getSlotColor() {return slotColor;}

    ///Reset
    public void reset() {
        this.hasBall = false;
        this.slotColor = BallColors.UNKNOWN;
    }
}