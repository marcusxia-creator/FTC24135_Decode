package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

@Disabled
public class LimelightTest {

    private RobotHardware robot;
    private Turret turret;

    private double conversionFactor;

    private final double THETA = Math.atan(turret_Center_Y_Offset / turret_Center_X_Offset);
    private final double turretCenterOffsetLength = Math.hypot(turret_Center_Y_Offset, turret_Center_X_Offset);

    public LimelightTest(RobotHardware robot, Turret turret) {
        this.robot = robot;
        this.turret = turret;
    }

    public void initLimelight(int apriltagID) {
        if (apriltagID == 24) {
            robot.limelight3A.pipelineSwitch(0);
        }
    }

    public void start() {
        robot.limelight3A.start();
    }

    public Pose2D updateTagMT2OFFSET(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH) {
            conversionFactor = 39.3700787;
        }
        else if (distanceUnit == DistanceUnit.MM) {
            conversionFactor = 1000;
        }
        else {
            throw new IllegalArgumentException("Distance Unit can only be in INCH or MM");
        }
        double yaw = turret.getTurretMotorAngle() + robot.pinpoint.getHeading(AngleUnit.DEGREES);//robot.external_imu.getAngularOrientation().firstAngle;
        robot.limelight3A.updateRobotOrientation(yaw);
        LLResult llResult = robot.limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D robotPose3D = llResult.getBotpose_MT2();
            double normalizedYaw = Math.toRadians(robotPose3D.getOrientation().getYaw() - 90);
            double yOffSet = Math.cos(normalizedYaw) * (0.1778 * conversionFactor);
            double xOffSet = Math.sin(normalizedYaw) * (0.1778 * conversionFactor);
            //return new Pose2D(distanceUnit, ((robotPose3D.getPosition().x * conversionFactor) - xOffSet), ((robotPose3D.getPosition().y * conversionFactor) - yOffSet), AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw());
            return new Pose2D (distanceUnit, xOffSet, yOffSet, AngleUnit.DEGREES, normalizedYaw);
        }

        return null;
    }

    public Pose2D updateTagMT2OFFSET2(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH) {
            conversionFactor = 39.3700787;
        }
        else if (distanceUnit == DistanceUnit.MM) {
            conversionFactor = 1000;
        }
        else {
            throw new IllegalArgumentException("Distance Unit can only be in INCH or MM");
        }

        double turretYaw = THETA - robot.pinpoint.getHeading(AngleUnit.RADIANS);
        double yOffSet = Math.cos(turretYaw) * (turretCenterOffsetLength * conversionFactor);
        double xOffSet = Math.sin(turretYaw) * (turretCenterOffsetLength * conversionFactor);
        //return new Pose2D(distanceUnit, ((robotPose3D.getPosition().x * conversionFactor) - xOffSet), ((robotPose3D.getPosition().y * conversionFactor) - yOffSet), AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw());
        return new Pose2D (distanceUnit, xOffSet, yOffSet, AngleUnit.DEGREES, turretYaw);
    }


    public Pose2D updateTagMT2(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH) {
            conversionFactor = 39.3700787;
        }
        else if (distanceUnit == DistanceUnit.MM) {
            conversionFactor = 1000;
        }
        else {
            throw new IllegalArgumentException("Distance Unit can only be in INCH or MM");
        }
        double yaw = turret.getTurretMotorAngle() + robot.pinpoint.getHeading(AngleUnit.DEGREES);//robot.external_imu.getAngularOrientation().firstAngle;
        robot.limelight3A.updateRobotOrientation(yaw);
        LLResult llResult = robot.limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D robotPose3D = llResult.getBotpose_MT2();
            double yOffSet = Math.sin(robotPose3D.getOrientation().getYaw(AngleUnit.DEGREES) - 90) * (0.1778 * conversionFactor);
            double xOffSet = Math.cos(robotPose3D.getOrientation().getYaw(AngleUnit.DEGREES) - 90) * (0.1778 * conversionFactor);
            return new Pose2D(distanceUnit, ((robotPose3D.getPosition().x * conversionFactor)), ((robotPose3D.getPosition().y * conversionFactor)), AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw());
            //return new Pose2D (distanceUnit, xOffSet, yOffSet, AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw(AngleUnit.DEGREES));
        }

        return null;
    }

    public Pose2D turretCenter(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH) {
            conversionFactor = 39.3700787;
        }
        else if (distanceUnit == DistanceUnit.MM) {
            conversionFactor = 1000;
        }
        else {
            throw new IllegalArgumentException("Distance Unit can only be in INCH or MM");
        }
        double yaw = turret.getTurretMotorAngle() + robot.pinpoint.getHeading(AngleUnit.DEGREES);//robot.external_imu.getAngularOrientation().firstAngle;
        robot.limelight3A.updateRobotOrientation(yaw);
        LLResult llResult = robot.limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D robotPose3D = llResult.getBotpose_MT2();
            double normalizedYaw = Math.toRadians(robotPose3D.getOrientation().getYaw() - 90);
            double yOffSet = Math.cos(normalizedYaw) * (0.1778 * conversionFactor);
            double xOffSet = Math.sin(normalizedYaw) * (0.1778 * conversionFactor);
            //return new Pose2D(distanceUnit, ((robotPose3D.getPosition().x * conversionFactor) - xOffSet), ((robotPose3D.getPosition().y * conversionFactor) - yOffSet), AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw());
            return new Pose2D (distanceUnit, (robotPose3D.getPosition().x * conversionFactor + xOffSet), (robotPose3D.getPosition().y * conversionFactor - yOffSet), AngleUnit.DEGREES, normalizedYaw);
        }

        return null;
    }

    public Pose2D updateTagMT2NORMALIZED(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH) {
            conversionFactor = 39.3700787;
        }
        else if (distanceUnit == DistanceUnit.MM) {
            conversionFactor = 1000;
        }
        else {
            throw new IllegalArgumentException("Distance Unit can only be in INCH or MM");
        }
        double yaw = turret.getTurretMotorAngle() + robot.pinpoint.getHeading(AngleUnit.DEGREES);//robot.external_imu.getAngularOrientation().firstAngle;
        robot.limelight3A.updateRobotOrientation(yaw);
        LLResult llResult = robot.limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D robotPose3D = llResult.getBotpose_MT2();
            double normalizedYaw = Math.toRadians(robotPose3D.getOrientation().getYaw(AngleUnit.DEGREES) - 90);
            double yOffSet = Math.cos(normalizedYaw) * (0.1778 * conversionFactor);
            double xOffSet = Math.sin(normalizedYaw) * (0.1778 * conversionFactor);
            double turretYaw = THETA - robot.pinpoint.getHeading(AngleUnit.RADIANS);
            double turretYOffSet = Math.cos(turretYaw) * (turretCenterOffsetLength * conversionFactor);
            double turretXOffSet = Math.sin(turretYaw) * (turretCenterOffsetLength * conversionFactor);
            return new Pose2D(distanceUnit, (robotPose3D.getPosition().x * conversionFactor + xOffSet + turretXOffSet), (robotPose3D.getPosition().y * conversionFactor - yOffSet - turretYOffSet), AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw());
            //return new Pose2D (distanceUnit, xOffSet, yOffSet, AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw(AngleUnit.DEGREES));
        }

        return null;
    }

    public double getTx() {
        double yaw = turret.getTurretMotorAngle() + robot.pinpoint.getHeading(AngleUnit.DEGREES);//robot.external_imu.getAngularOrientation().firstAngle;
        robot.limelight3A.updateRobotOrientation(yaw);
        LLResult llResult = robot.limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTx();
        }

        return 0.0;
    }

    public double getTy() {
        double yaw = turret.getTurretMotorAngle() + robot.pinpoint.getHeading(AngleUnit.DEGREES);//robot.external_imu.getAngularOrientation().firstAngle;
        robot.limelight3A.updateRobotOrientation(yaw);
        LLResult llResult = robot.limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getTy();
        }

        return 0.0;
    }
}
