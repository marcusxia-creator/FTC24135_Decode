package org.firstinspires.ftc.teamcode.TeleOps;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Disabled
public class Limelight {

    private RobotHardware robot;

    private double conversionFactor;

    public Limelight(RobotHardware robot) {
        this.robot = robot;
    }

    public void initLimelight(int apriltagID) {
        if (apriltagID == 24) {
            robot.limelight3A.pipelineSwitch(0);
        }
    }

    public void start() {
        robot.limelight3A.start();
    }

    public Pose2D updateTagMT2(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH) {
            conversionFactor = 39.3700787;
        }
        if (distanceUnit == DistanceUnit.MM) {
            conversionFactor = 1000;
        }
        else {
            throw new IllegalArgumentException("Distance Unit can only be in INCH or MM");
        }
        double yaw = robot.external_imu.getAngularOrientation().firstAngle;
        robot.limelight3A.updateRobotOrientation(yaw);
        LLResult llResult = robot.limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D robotPose3D = llResult.getBotpose_MT2();
            return new Pose2D(distanceUnit, robotPose3D.getPosition().x * conversionFactor, robotPose3D.getPosition().y * conversionFactor, AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw());
        }

        return null;
    }
}
