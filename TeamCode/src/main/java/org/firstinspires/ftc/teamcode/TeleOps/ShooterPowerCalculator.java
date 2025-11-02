package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class ShooterPowerCalculator {

    private RobotHardware robot;

    private Pose2D currentRobotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private double distance;

    private Pose2D redGoalPose = new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, -45);
    private Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, 70, 70, AngleUnit.DEGREES, -45);

    public ShooterPowerCalculator (RobotHardware robot) {
        this.robot = robot;
    }

    public void init() {
        robot.initOdo();
    }

    public void updateDistance() {
        robot.odo.update();
        distance = Math.sqrt(Math.pow(robot.odo.getPosX(DistanceUnit.INCH) - redGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.odo.getPosY(DistanceUnit.INCH) - redGoalPose.getY(DistanceUnit.INCH), 2));
    }

    public double getPower() {
        updateDistance();
        return a * Math.pow(distance, exp);
    }
}
