package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.LUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class LUTPowerCalculator {

    private RobotHardware robot;
    private PIDController pidController;

    private double distance;
    private int zone = 0;

    private double tickToRPM = (double) 60 / 28;

    private int maxVelocity = 4500;

    private final Pose2D redGoalPose = new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, -45);
    private final Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, -70, -70, AngleUnit.DEGREES, 45);

    private Pose2D actualGoalPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    private final double p = 5, i = 0, d = 0;

    LUT<Integer, Integer> targetRPM = new LUT<Integer, Integer>()
    {{
        add(3, 3000);//far zone
        add(2, 2500); //mid zone
        add(1, 2000); //near zone
        add(0, 1500); //Not in shooting zone

    }};

    public LUTPowerCalculator(RobotHardware robot) {
        this.robot = robot;
        pidController = new PIDController(p, i, d);
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
        if (distance > close && distance <= mid) {
            zone = 1;
        }
        else if (distance > mid && distance <= far) {
            zone = 2;
        }
        else if (distance > far && distance <= farEdge) {
            zone = 3;
        }
        else {
            zone = 0;
        }

        double current = (robot.shooterMotor.getVelocity() * tickToRPM) / maxVelocity;
        double target = targetRPM.get(zone) / maxVelocity;

        return pidController.calculate(current, target);
    }

    public double getAngle() {
        double tangent = (actualGoalPose.getY(DistanceUnit.INCH) - robot.pinpoint.getPosY(DistanceUnit.INCH)) / distance;
        return Math.toDegrees(Math.atan(tangent));
    }
}
