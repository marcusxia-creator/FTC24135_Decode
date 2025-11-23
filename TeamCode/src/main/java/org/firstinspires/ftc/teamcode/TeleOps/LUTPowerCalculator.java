package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.util.LUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class LUTPowerCalculator {

    private RobotHardware robot;

    private double distance;
    private int zone = 0;

    private final Pose2D redGoalPose = new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, -45);
    private final Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, -70, -70, AngleUnit.DEGREES, -45);

    private Pose2D actualGoalPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    LUT<Integer, Double> power = new LUT<Integer, Double>()
    {{
        add(0, 0.3);
        add(3, 0.75);
        add(2, 0.5);
        add(1, 0.2);
    }};

    public LUTPowerCalculator(RobotHardware robot) {
        this.robot = robot;
    }

    public void setAlliance (boolean isRedAlliance) {
        if (isRedAlliance) {
            actualGoalPose = redGoalPose;
        }

        if (!isRedAlliance) {
            actualGoalPose = blueGoalPose;
        }
    }

    public double getDistance() {
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH), 2));
        return distance;
    }

    public double getPower() {
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH), 2));
        if (distance > 53 && distance <= 68.6) {
            zone = 1;
        }
        if (distance > 68.6 && distance <= 84.2) {
            zone = 2;
        }
        if (distance > 84.2 && distance <= 100) {
            zone = 3;
        }
        else {
            zone = 0;
        }

        return power.get(zone);
    }

    public double getAngle() {
        double tangent = (actualGoalPose.getY(DistanceUnit.INCH) - robot.pinpoint.getPosY(DistanceUnit.INCH)) / distance;
        return Math.toDegrees(Math.atan(tangent));
    }
}
