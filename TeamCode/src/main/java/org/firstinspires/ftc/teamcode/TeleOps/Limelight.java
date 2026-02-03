package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.aimingAngleThrehold;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turretCameraRadius;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_X_Offset;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_Y_Offset;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight {

    private RobotHardware robot;
    private Turret turret;
    private double conversionFactor;
    private final double THETA = Math.atan(turret_Center_Y_Offset / turret_Center_X_Offset);
    private final double turretCenterOffsetLength = Math.hypot(turret_Center_Y_Offset, turret_Center_X_Offset);


    public Limelight(RobotHardware robot, Turret turret) {
        this.robot = robot;
        this.turret = turret;
    }

    /// init limelight
    public void initLimelight(int apriltagID) {
        if (apriltagID == 24) {
            robot.limelight.pipelineSwitch(0);
        }
    }

    /// start limelight
    public void start() {
        robot.limelight.start();
    }

    /// Determine Limelight result
    public boolean llresult(){
        return robot.limelight.getLatestResult()!=null;
    }

    //============================================================
    // Use Limelight Tx Angle to filter the llresult
    // generate Tag angle for turret angle correction
    //============================================================
    public double getTargetX() {
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.getFiducialResults() != null) {
            if (result.getTx()<aimingAngleThrehold)
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    // Horizontal angular offset (degrees)
                    return fiducial.getTargetXDegrees();
                }
            else{
                return 0.0;
            }
        }
        // No valid AprilTag
        return 0.0;
    }

    //============================================================
    // Generate Pose 2D with offset correction
    //============================================================
    public Output normalizedPose2D(DistanceUnit distanceUnit) {
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
        robot.limelight.updateRobotOrientation(yaw);
        LLResult llResult = robot.limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D robotPose3D = llResult.getBotpose_MT2();
            double normalizedYaw = Math.toRadians(robotPose3D.getOrientation().getYaw(AngleUnit.DEGREES) - 90);
            double yOffSet = Math.sin(normalizedYaw) * (turretCameraRadius * conversionFactor); // unit M, 0.1778 is the radius of the turret center to limelight
            double xOffSet = Math.cos(normalizedYaw) * (turretCameraRadius * conversionFactor);
            double turretYaw = THETA - robot.pinpoint.getHeading(AngleUnit.RADIANS);
            double turretYOffSet = Math.sin(turretYaw) * (turretCenterOffsetLength * conversionFactor);
            double turretXOffSet = Math.cos(turretYaw) * (turretCenterOffsetLength * conversionFactor);
            Pose2D robotPose = new Pose2D(distanceUnit, (robotPose3D.getPosition().x * conversionFactor + (xOffSet + turretXOffSet)), (robotPose3D.getPosition().y * conversionFactor - (yOffSet + turretYOffSet)), AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw());
            return new Output(robotPose, xOffSet, yOffSet, turretXOffSet, turretYOffSet);
        }
        return null;
    }

    /**
     * Output class to return Pose, offset, ect.
     */
    public static class Output {
        public final Pose2D robotPose;
        public final double cameraXOffset;
        public final double cameraYOffset;
        public final double turretXOffset;
        public final double turretYOffset;

        public Output(Pose2D robotPose, double cameraXOffset, double cameraYOffset, double turretXOffSet, double turretYOffset) {
            this.robotPose = robotPose;
            this.cameraXOffset = cameraXOffset;
            this.cameraYOffset = cameraYOffset;
            this.turretXOffset = turretXOffSet;
            this.turretYOffset = turretYOffset;
        }
    }

}
