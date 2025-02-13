package org.firstinspires.ftc.teamcode.Auto.AprilTagAuto;

import java.util.HashMap;
import java.util.Map;

public class TestHarshMap {

    public static void main(String[] args) {

        Map<Integer, Double[]> AprilTagID = new HashMap<>();
        Double[] AprilTagCoordinateArray = new Double[]{5.5, 7.6};
        Double[] AprilTagCoordinateArray2 = new Double[] {3.4, 6.6};

        int ID = 1;
        int ID2 = 2;

        AprilTagID.put(ID, AprilTagCoordinateArray);
        AprilTagID.put(ID2, AprilTagCoordinateArray2);

        double IDx = AprilTagID.get(ID)[0];
        double ID2x = AprilTagID.get(ID2)[0];

    }

}
