package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.EnumMap;

public class ShooterDiscreteZonePowerTable {

    public enum SHOOTRANGE {
        NEAR,
        MIDDLE,
        FAR
    }

    private final EnumMap<SHOOTRANGE, Double> powerTable = new EnumMap<>(SHOOTRANGE.class);

    public ShooterDiscreteZonePowerTable() {
        powerTable.put(SHOOTRANGE.NEAR, 0.75);
        powerTable.put(SHOOTRANGE.MIDDLE, 0.73);
        powerTable.put(SHOOTRANGE.FAR, 0.80);
    }


    public double getPower(double distance) {
        if (distance < 100 && distance > 80) {
            return powerTable.get(SHOOTRANGE.FAR);
        }
        else if (distance < 80 && distance > 70) {
            return powerTable.get(SHOOTRANGE.MIDDLE);
        }
        else if (distance < 70 && distance > 50){
            return powerTable.get(SHOOTRANGE.NEAR);
        }
        else {
            return 0.0;
        }
    }
}