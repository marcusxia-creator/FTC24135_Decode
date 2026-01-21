package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_X_Offset;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_Y_Offset;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {

    private RobotHardware robot;
    private Turret turret;

    private final double turretRadius = 0.1778;
    private final double turretCenterOffsetLength = Math.hypot(turret_Center_Y_Offset, turret_Center_X_Offset);
    private final double THETA = Math.atan(turret_Center_Y_Offset / turret_Center_X_Offset);

    private final int meterToMm = 1000;
    private double conversionFactor;

    private int stableCount = 0;

    /// Tuning Factor --------------------------------------------
    private long maxStalenessMs = 120;

    private int goodTagCount = 2;

    private double maxTxForHighTrust = 12.0;
    private double maxTyForHighTrustDeg = 12.0;

    private double alphaMin = 0.03;
    private double alphaMax = 0.25;

    private double resetMinQuality = 0.85;
    private int resetStableFrames = 4;
    private double resetMinErrorInches = 6.0;

    private double angleAdjustmentTy = -20;


    public Limelight(RobotHardware robot, Turret turret) {
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

    public void update() {
        //return Output;
    }

    public Output normalizedPose2D(DistanceUnit distanceUnit) {

        double yaw = turret.getTurretMotorAngle() + robot.pinpoint.getHeading(AngleUnit.DEGREES);//robot.external_imu.getAngularOrientation().firstAngle;
        robot.limelight3A.updateRobotOrientation(yaw);
        LLResult llResult = robot.limelight3A.getLatestResult();

        if (llResult == null && !llResult.isValid()) {
            stableCount = 0;
            return null;
        }

        long staleness = llResult.getStaleness();

        if (staleness > maxStalenessMs) {
            stableCount = 0;
            return null;
        }

        int tagCount = (llResult.getFiducialResults() == null) ? 0 : llResult.getFiducialResults().size();
        double tx = llResult.getTx();
        double ty = llResult.getTy();


        Pose3D robotPose3D = llResult.getBotpose_MT2();

        if (robotPose3D == null) {
            stableCount = 0;
            return null;
        }

        double normalizedYaw = Math.toRadians(robotPose3D.getOrientation().getYaw(AngleUnit.DEGREES) - 90);
        double yOffSet = Math.sin(normalizedYaw) * (turretRadius * meterToMm);
        double xOffSet = Math.cos(normalizedYaw) * (turretRadius * meterToMm);
        double turretYaw = THETA - robot.pinpoint.getHeading(AngleUnit.RADIANS);
        double turretYOffSet = Math.sin(turretYaw) * (turretCenterOffsetLength * meterToMm);
        double turretXOffSet = Math.cos(turretYaw) * (turretCenterOffsetLength * meterToMm);
        llResult.getStddevMt2();

        Pose2D robotPose = new Pose2D(DistanceUnit.MM, (robotPose3D.getPosition().x * meterToMm + (xOffSet + turretXOffSet)), (robotPose3D.getPosition().y * meterToMm - (yOffSet + turretYOffSet)), AngleUnit.DEGREES, robotPose3D.getOrientation().getYaw());

        double quality = computeQuality(tagCount, tx, ty, staleness);

        double alpha = alphaMin + (alphaMax - alphaMin) * quality;

        Pose2D fusedPose = fusePose(robot.pinpoint.getPosition(), robotPose, alpha);

        boolean didRest = false;
        double error = Math.hypot(robotPose.getX(DistanceUnit.MM) - robot.pinpoint.getPosX(DistanceUnit.MM),
                robotPose.getY(DistanceUnit.MM) - robot.pinpoint.getPosY(DistanceUnit.MM));

        if (quality >= resetMinQuality && error >= resetMinErrorInches && tagCount >= goodTagCount) {
            stableCount++;
            if (stableCount >= resetStableFrames) {
                stableCount = 0;
                didRest = true;

                fusedPose = new Pose2D(DistanceUnit.MM, robotPose.getX(DistanceUnit.MM), robotPose.getY(DistanceUnit.MM), AngleUnit.DEGREES, robot.pinpoint.getHeading(AngleUnit.DEGREES));
            }
            else {
                stableCount = 0;
            }
        }

        if (distanceUnit == DistanceUnit.INCH) {
            conversionFactor = 0.0393700787;
        }
        else if (distanceUnit == DistanceUnit.MM) {
            conversionFactor = 1;
        }
        else {
            throw new IllegalArgumentException("Distance Unit can only be in INCH or MM");
        }

        Pose2D finalPose = new Pose2D(distanceUnit, fusedPose.getX(DistanceUnit.MM) * conversionFactor, fusedPose.getY(DistanceUnit.MM) * conversionFactor, AngleUnit.DEGREES, fusedPose.getHeading(AngleUnit.DEGREES));
        return new Output(finalPose, finalPose.getX(DistanceUnit.MM), finalPose.getY(DistanceUnit.MM));
    }

    private double computeQuality(int tagCount, double tx, double ty, long stalenessMs) {
        double tagScore = (tagCount >= goodTagCount) ? 1.0 : (tagCount == 1 ? 0.55 : 0.0);

        double txScore = 1.0 - Range.clip(Math.abs(tx) / maxTxForHighTrust, 0.0, 1.0);
        double tyScore = 1.0 - Range.clip(Math.abs(ty + angleAdjustmentTy) / maxTyForHighTrustDeg, 0.0, 1.0);
        double centerScore = 0.5 * (txScore + tyScore);

        double freshnessScore = 1.0 - Range.clip(stalenessMs / (double) maxStalenessMs, 0.0, 1.0);

        double overallScore = 0.55 * tagScore + 0.30 * centerScore + 0.15 * freshnessScore;
        return Range.clip(overallScore, 0.0, 1.0);
    }

    private Pose2D fusePose(Pose2D odometryPose, Pose2D visionPose, double alpha) {
        double x = odometryPose.getX(DistanceUnit.MM) + alpha * (visionPose.getX(DistanceUnit.MM));
        double y = odometryPose.getY(DistanceUnit.MM) + alpha * (visionPose.getY(DistanceUnit.MM));
        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, odometryPose.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Output class to return Pose, offset, ect.
     */
    public static class Output {
        public Pose2D robotPose;
        public double cameraXOffset;
        public double cameraYOffset;

        public Output(Pose2D robotPose, double cameraXOffset, double cameraYOffset) {
            this.robotPose = robotPose;
            this.cameraXOffset = cameraXOffset;
            this.cameraYOffset = cameraYOffset;
        }
    }

}
