package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.ArrayList;
import java.util.List;

public class SharedBallList {

    private final List<Ball> balls = new ArrayList<>();

    public SharedBallList(double[] slotAngles) {
        for (int i = 0; i < slotAngles.length; i++) {
            balls.add(new Ball(i, slotAngles[i]));
        }
    }

    public List<Ball> getBalls() {
        return balls;
    }
}
