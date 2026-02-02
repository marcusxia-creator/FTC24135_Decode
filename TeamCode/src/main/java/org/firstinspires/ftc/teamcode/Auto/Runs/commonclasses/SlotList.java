package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class SlotList {
    private final List<SpindexerSlot> slotList = new ArrayList<>();
    private final double[] slotAngles = {
            spindexerSlot1,
            spindexerSlot2,
            spindexerSlot3
    };

    public SlotList () {
        for (int i=0;i < slotAngles.length; i++){
            slotList.add(new SpindexerSlot(i,slotAngles[i]));
        }
    }

    public int getSlotCount() {
        return slotList.size();
    }

    public SpindexerSlot getSlot(int slotPosition) {
        if (slotPosition < 0 || slotPosition >= slotList.size()) return null;
        return slotList.get(slotPosition);
    }

    public void setSlotBall(int slotPosition, BallColors color) {
        SpindexerSlot slot = getSlot(slotPosition);
        if (slot != null) {
            slot.setBall(true);
            slot.setSlotColor(color);
        }
    }

    public void setHasBall(int slotPosition, Boolean hasBall) {
        SpindexerSlot slot = slotList.get(slotPosition);
        if (slot != null) {
            slot.setBall(hasBall);
        }
    }

    public void reset() {
        for (SpindexerSlot slot : slotList) {
            slot.reset();
        }
    }

    public List<SpindexerSlot> getSlotList() {
        return slotList;
    }
}

///Pseudocode for Learning (Used in Main Class)
/*
slotList = SlotList.getSlotList();
for (int i=0;i<orderList.length;i++){
    ball = slotList.get(i);
    slotAngle = ball.getSlotAngle();
}

 */