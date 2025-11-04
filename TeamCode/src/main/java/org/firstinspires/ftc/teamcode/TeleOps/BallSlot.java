package org.firstinspires.ftc.teamcode.TeleOps;

public class BallSlot {

    private final int slotPosition;   // Slot index (0, 1, 2)
    private final double slotAngle;   // Servo position for this slot
    public boolean hasBall;          // True if a ball is present
    public BallColor color;          // Enum color (RED, BLUE, YELLOW, UNKNOWN)

    // --- Constructor ---
    public BallSlot(int slotPosition, double slotAngle) {
        this.slotPosition = slotPosition;
        this.slotAngle = slotAngle;
        this.hasBall = false;
        this.color = BallColor.UNKNOWN;  // Start empty
    }

    // --- Getters ---
    public int getSlotPosition() { return slotPosition; }
    public double getSlotAngle() { return slotAngle; }
    public boolean hasBall() { return hasBall; }
    public BallColor getColor() { return color; }

    // --- Setters ---
    public void setHasBall(boolean hasBall) { this.hasBall = hasBall; }
    public void setBallColor(BallColor color) {
        if (this.color != color) this.color = color;
    }

    public void reset() {
        hasBall = false;
        color = BallColor.UNKNOWN;
    }
}