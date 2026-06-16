package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

public class AutoSpindexerContext {
    public int currentGreenSlot = -1;
    public int targetGreenSlot = 0;
    public int shootingInitSlot = 0;

    public boolean shooterStarted = false;
    public boolean intakeShouldStop = false;

    public void updateShootingInitSlot() {
        if (currentGreenSlot == -1) {
            shootingInitSlot = 0;
        } else {
            shootingInitSlot = Math.floorMod(currentGreenSlot - targetGreenSlot, 3);
        }
    }
}
