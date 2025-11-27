package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.EnumMap;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class ShooterDiscreteZonePowerTable {

    public enum SHOOTRANGE {
        NEAR,
        MIDDLE,
        FAR
    }

    private final EnumMap<SHOOTRANGE, Double> powerTable = new EnumMap<>(SHOOTRANGE.class);

    public ShooterDiscreteZonePowerTable() {
        powerTable.put(SHOOTRANGE.NEAR, closePower);
        powerTable.put(SHOOTRANGE.MIDDLE, midPower);
        powerTable.put(SHOOTRANGE.FAR, farPower);
    }


    public double getPower(double distance) {
        if (distance < farEdge && distance > far) {
            return powerTable.get(SHOOTRANGE.FAR);
        }
        else if (distance < far && distance > mid) {
            return powerTable.get(SHOOTRANGE.MIDDLE);
        }
        else if (distance < mid && distance > close){
            return powerTable.get(SHOOTRANGE.NEAR);
        }
        else {
            return ozPower;
        }
    }
}