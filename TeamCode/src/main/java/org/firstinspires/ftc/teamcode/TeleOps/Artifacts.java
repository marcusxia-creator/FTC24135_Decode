package org.firstinspires.ftc.teamcode.TeleOps;

import androidx.annotation.NonNull;

import java.util.HashMap;

public class Artifacts {

    HashMap<Integer, Integer> artifact = new HashMap<>();

    /**
     * color:
     *  0 = none
     *  1 = green
     *  2 = purple
     *  3 = undefined
     */

    public void init() {
        artifact.put(1, 0);
        artifact.put(2, 0);
        artifact.put(3, 0);
    }

    public void putArtifacts(int slotNumber, int color) {
        artifact.put(slotNumber, color);
    }

    public double returnArtifacts(int slotNumber) {
        return artifact.get(slotNumber);
    }

    public void clearSlots(int slotNumber) {
        artifact.put(slotNumber, 0);
    }
}
