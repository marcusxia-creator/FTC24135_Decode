package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class ShooterPowerCalculator {

    private RobotHardware robot;
    private GoBildaPinpointDriver odo;
    private HardwareMap hardwareMap;

    private Pose2D currentRobotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private double distance;

    private Pose2D redGoalPose = new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, -45);
    private Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, 70, 70, AngleUnit.DEGREES, -45);

    public ShooterPowerCalculator (RobotHardware robot, HardwareMap hardwareMap) {
        this.robot = robot;
        this.hardwareMap = hardwareMap;
        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void init() {
        odo.setOffsets(-149.225, -165.1,DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public void updateDistance() {
        odo.update();
        distance = Math.sqrt(Math.pow(odo.getPosX(DistanceUnit.INCH) - redGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(odo.getPosY(DistanceUnit.INCH) - redGoalPose.getY(DistanceUnit.INCH), 2));
    }

    /*public double getPower() {

        return a * Math.pow(distance, exp);
    }

     */
}
