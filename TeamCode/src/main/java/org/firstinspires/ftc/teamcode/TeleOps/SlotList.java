package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.TeleOps.BallColor;

public class SlotList {

    // ===== Slot data =====
    private final List<BallSlot> slots = new ArrayList<>();

    /**
     * @param slotAngles servo positions (or angles) for each slot index
     */
    public SlotList(double[] slotAngles) {
        for (int i = 0; i < slotAngles.length; i++) {
            slots.add(new BallSlot(i, slotAngles[i]));
        }
    }

    // ===== Safe accessors =====

    /** Read-only view so outside code can't add/remove/reorder. */
    public List<BallSlot> getSlotsReadOnly() {
        return Collections.unmodifiableList(slots);
    }

    public BallSlot getSlot(int index) {
        return slots.get(index);
    }

    public int size() {
        return slots.size();
    }

    // ===== Core logic =====

    /** How many slots currently have a ball. */
    public int getBallCount() {
        int count = 0;
        for (BallSlot s : slots) {
            if (s.hasBall()) count++;
        }
        return count;
    }

    public boolean isEmpty() {
        return getBallCount() == 0;
    }

    public boolean isFull() {
        return getBallCount() == slots.size();
    }

    /** Returns the first empty slot index, or -1 if none. */
    public int findNextEmptySlot() {
        for (BallSlot s : slots) {
            if (!s.hasBall()) return s.getSlotPosition();
        }
        return -1;
    }

    /** Returns the first slot index that matches the given color, or -1 if not found. */
    public double findFirstByColor(BallColor color) {
        for (BallSlot s : slots) {
            if (s.hasBall() && s.getColor() == color) return s.getSlotAngle();
        }
        return -1;
    }
    /** Return next ball slot */
    public int findAnySlotWithBall() {
        for (BallSlot s : slots) {
            if (s.hasBall()) {
                return s.getSlotPosition();
            }
        }
        return -1; // none found
    }

    /** Set slot content by color. UNKNOWN means empty. Keeps hasBall consistent. */
    public void setSlotColor(int index, BallColor color) {
        BallSlot s = slots.get(index);
        s.setBallColor(color);
        s.setHasBall(color != BallColor.UNKNOWN);
    }

    /** Explicitly set whether a slot has a ball. If set false, color resets to UNKNOWN. */
    public void setSlotHasBall(int index, boolean hasBall) {
        BallSlot s = slots.get(index);
        s.setHasBall(hasBall);
        if (!hasBall) s.setBallColor(BallColor.UNKNOWN);
    }

    /** Reset all slots to empty/unknown. */
    public void resetAll() {
        for (BallSlot s : slots) s.reset();
    }

    // ===== Nested BallSlot class =====
    public static class BallSlot {

        private final int slotPosition;   // Slot index (0..n-1)
        private final double slotAngle;   // Servo position/angle for this slot

        private boolean hasBall;          // True if a ball is present
        private BallColor color;          // RED, BLUE, YELLOW, UNKNOWN

        public BallSlot(int slotPosition, double slotAngle) {
            this.slotPosition = slotPosition;
            this.slotAngle = slotAngle;
            this.hasBall = false;
            this.color = BallColor.UNKNOWN;
        }

        public int getSlotPosition() { return slotPosition; }
        public double getSlotAngle() { return slotAngle; }
        public boolean hasBall() { return hasBall; }
        public BallColor getColor() { return color; }

        public void setHasBall(boolean hasBall) {
            this.hasBall = hasBall;
        }

        public void setBallColor(BallColor color) {
            this.color = color;
        }

        public void reset() {
            hasBall = false;
            color = BallColor.UNKNOWN;
        }
    }

}
