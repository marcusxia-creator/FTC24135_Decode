package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_X_Offset;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_Y_Offset;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class Limelight {

    private RobotHardware robot;
    private Turret turret;
    private VisionProcessing visionProcessing;

    private final double turretRadius = 0.1778;
    private final double turretCenterOffsetLength = Math.hypot(turret_Center_Y_Offset, turret_Center_X_Offset);
    private final double THETA = Math.atan(turret_Center_Y_Offset / turret_Center_X_Offset);

    private final int meterToMm = 1000;
    private double conversionFactor;

    private int stableCount = 0;

    /// Tuning Factor --------------------------------------------
    private final long maxStalenessMs = 120;

    private final int goodTagCount = 1;

    private final double maxTxForHighTrust = 12.0;
    private final double maxTyForHighTrustDeg = 12.0;

    private final double alphaMin = 0.03;
    private final double alphaMax = 0.25;

    private final double resetMinQuality = 0.85;
    private final int resetStableFrames = 4;
    private final double resetMinErrorInches = 6.0;

    private final double angleAdjustmentTy = -20;


    public Limelight(RobotHardware robot, Turret turret) {
        this.robot = robot;
        this.turret = turret;

        visionProcessing = new VisionProcessing();
    }

    public void initLimelight(int apriltagID) {
        visionProcessing.clear();

        if (apriltagID == 24) {
            robot.limelight3A.pipelineSwitch(0);
        }
    }

    public void start() {
        robot.limelight3A.start();
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


        Pose3D visionPose3D = llResult.getBotpose_MT2();

        if (visionPose3D == null) {
            stableCount = 0;
            return null;
        }

        Pose2D robotVisionPose2D = normalizedRobotPose(visionPose3D);

        double quality = computeQuality(tagCount, tx, ty, staleness);

        double alpha = alphaMin + (alphaMax - alphaMin) * quality;

        VisionProcessing.Stats stats = visionProcessing.updateAndGet(robot.pinpoint.getPosition(), robotVisionPose2D);

        Pose2D fusedPose = fusePose(robot.pinpoint.getPosition(), robotVisionPose2D, alpha);

        boolean didRest = false;
        double error = Math.hypot(robotVisionPose2D.getX(DistanceUnit.MM) - robot.pinpoint.getPosX(DistanceUnit.MM),
                robotVisionPose2D.getY(DistanceUnit.MM) - robot.pinpoint.getPosY(DistanceUnit.MM));

        if (quality >= resetMinQuality && error >= resetMinErrorInches && tagCount >= goodTagCount) {
            stableCount++;
            if (stableCount >= resetStableFrames) {
                stableCount = 0;
                didRest = true;

                fusedPose = new Pose2D(DistanceUnit.MM, robotVisionPose2D.getX(DistanceUnit.MM), robotVisionPose2D.getY(DistanceUnit.MM), AngleUnit.DEGREES, robot.pinpoint.getHeading(AngleUnit.DEGREES));
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
        return new Output(finalPose, robotVisionPose2D, robot.pinpoint.getPosition(),
                tx, ty, quality, alpha,
                error, staleness, tagCount, didRest);
    }

    private Pose2D normalizedRobotPose(Pose3D visionPose3D) {
        double normalizedYaw = Math.toRadians(visionPose3D.getOrientation().getYaw(AngleUnit.DEGREES) - 90);
        double yOffSet = Math.sin(normalizedYaw) * (turretRadius * meterToMm);
        double xOffSet = Math.cos(normalizedYaw) * (turretRadius * meterToMm);
        double turretYaw = THETA - robot.pinpoint.getHeading(AngleUnit.RADIANS);
        double turretYOffSet = Math.sin(turretYaw) * (turretCenterOffsetLength * meterToMm);
        double turretXOffSet = Math.cos(turretYaw) * (turretCenterOffsetLength * meterToMm);

        return new Pose2D(DistanceUnit.MM, (visionPose3D.getPosition().x * meterToMm + (xOffSet + turretXOffSet)), (visionPose3D.getPosition().y * meterToMm - (yOffSet + turretYOffSet)), AngleUnit.DEGREES, visionPose3D.getOrientation().getYaw());
    }

    private double computeQuality(int tagCount, double tx, double ty, long stalenessMs) {
        double tagScore = tagCount >= goodTagCount ? 1.0 : 0.0;

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
        public Pose2D fusedPose;
        public Pose2D visionPose;
        public Pose2D pinpointPose;
        public double tx;
        public double ty;
        public double quality;
        public double alpha;
        public double error;
        public double staleness;
        public double tagCount;
        public boolean didReset;

        public Output(Pose2D fusedPose, Pose2D visionPose, Pose2D pinpointPose,
                      double tx, double ty, double quality, double alpha,
                      double error, double staleness, double tagCount, boolean didReset) {
            this.fusedPose = fusedPose;
            this.visionPose = visionPose;
            this.pinpointPose = pinpointPose;
            this.tx = tx;
            this.ty = ty;
            this.quality = quality;
            this.alpha = alpha;
            this.error = error;
            this.staleness = staleness;
            this.tagCount = tagCount;
            this.didReset = didReset;
        }
    }

}
