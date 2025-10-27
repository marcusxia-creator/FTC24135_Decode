package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.ArrayList;
import java.util.List;

public class SharedBallList {
    private final List<Ball> balls = new ArrayList<>();

    public SharedBallList(double[] slotAngles) {
        for (int i = 0; i < slotAngles.length; i++) {
            balls.add(new Ball("Empty", i, slotAngles[i], false));
        }
    }

    public List<Ball> getBalls() { return balls; }

    public void resetAll() {
        for (Ball b : balls) {
            b.setHasBall(false);
            b.setBallColor("Empty");
        }
    }
}
