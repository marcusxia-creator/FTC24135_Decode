package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.ArrayList;
import java.util.List;

public class SlotList {

    private final List<BallSlot> ballSlots = new ArrayList<>();

    public SlotList(double[] slotAngles) {
        for (int i = 0; i < slotAngles.length; i++) {
            ballSlots.add(new BallSlot(i, slotAngles[i]));
        }
    }

    public List<BallSlot> getBalls() {
        return ballSlots;
    }
}
