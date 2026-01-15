package org.firstinspires.ftc.teamcode.TeleOps.Sensors;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightVisionFusion {
    ///  Output - for return result.
    public static class Output {
        public final boolean hasVision;
        public final double quality;      // 0..1
        public final long stalenessMs;
        @Nullable public final Pose2D visionPose;
        public final Pose2D fusedPose;
        public final boolean didPinpointReset;

        public Output(boolean hasVision, double quality, long stalenessMs,
                      @Nullable Pose2D visionPose, Pose2D fusedPose, boolean didPinpointReset) {
            this.hasVision = hasVision;
            this.quality = quality;
            this.stalenessMs = stalenessMs;
            this.visionPose = visionPose;
            this.fusedPose = fusedPose;
            this.didPinpointReset = didPinpointReset;
        }
    }

    // ---------------- Tunables ----------------
    // Units
    public double inchesPerMeter = 39.37007874015748;

    // Gate bad frames
    public long maxStalenessMs = 120;

    // Quality shaping
    public int goodTagCount = 2;         // prefer >=2 tags for strong trust
    public double maxTxForHighTrustDeg = 12.0;  // centered is more stable
    public double maxTyForHighTrustDeg = 12.0;

    // Fusion
    public double alphaMin = 0.03;       // minimum nudge
    public double alphaMax = 0.25;       // max nudge when quality is high

    // Outlier rejection (teleport gate)
    public double maxStepInches = 10.0;  // reject vision if it jumps too far vs current odom

    // Pinpoint reset conditions
    public double resetMinQuality = 0.85;
    public int resetStableFrames = 4;    // require N consecutive good frames
    public double resetMinErrorInches = 6.0; // only reset if odom drift is meaningful

    // ------------------------------------------

    private double filtX = Double.NaN;
    private double filtY = Double.NaN;
    private double filtH = Double.NaN; // radians (optional)
    private int stableCount = 0;

    /**
     * Main update: read MegaTag2, score it, filter it, fuse to odometry,
     * optionally "pinpoint reset" when stable & high-quality.
     *
     * @param limelight Limelight3A instance
     * @param imuYawDeg robot heading from Turret motor angle + heading (degrees, -180..180 ok)
     * @param odomPose  your current odometry pose (Pinpoint Pose2D, inches + radians)
     */
    public Output update(Limelight3A limelight, double imuYawDeg, Pose2D odomPose) {

        // MegaTag2 likes having robot yaw fed in each loop
        limelight.updateRobotOrientation(imuYawDeg);

        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) {
            stableCount = 0;
            return new Output(false, 0.0, Long.MAX_VALUE, null, odomPose, false);
        }

        long staleness = r.getStaleness();
        if (staleness > maxStalenessMs) {
            stableCount = 0;
            return new Output(false, 0.0, staleness, null, odomPose, false);
        }

        // Tag count + angles help estimate confidence
        int tagCount = (r.getFiducialResults() == null) ? 0 : r.getFiducialResults().size();
        double tx = r.getTx();
        double ty = r.getTy();


        Pose3D p = r.getBotpose_MT2();
        if (p == null) {
            stableCount = 0;
            return new Output(false, 0.0, staleness, null, odomPose, false);
        }

        // Vision pose from limelight (meters -> inches)
        double xIn = p.getPosition().x * inchesPerMeter;
        double yIn = p.getPosition().y * inchesPerMeter;

        // Keep heading from IMU (recommended). Convert to radians.
        double hRad = Math.toRadians(imuYawDeg);

        Pose2D rawVisionPose = new Pose2D(DistanceUnit.INCH,xIn, yIn, AngleUnit.RADIANS,hRad);

        // Gate: reject if vision pose is an outlier vs odometry
        double step = hypot(xIn - odomPose.getX(DistanceUnit.INCH), yIn - odomPose.getY(DistanceUnit.INCH));
        if (step > maxStepInches) {
            stableCount = 0;
            return new Output(false, 0.0, staleness, null, odomPose, false);
        }

        // Compute quality score 0..1
        double quality = computeQuality(tagCount, tx, ty, staleness);

        // Filter (EMA) raw vision x/y; use quality to pick smoothing factor
        // Higher quality -> higher alpha -> follows faster (less lag)
        double alpha = alphaMin + (alphaMax - alphaMin) * quality;

        Pose2D filtVision = filterVision(rawVisionPose, alpha);

        // Fuse: nudge odom toward filtered vision (X/Y only, keep odom heading as-is)
        Pose2D fused = fuseXYKeepHeading(odomPose, filtVision, alpha);

        // Pinpoint reset: only when very stable, very confident, and drift is meaningful
        boolean didReset = false;
        double err = hypot(filtVision.getX(DistanceUnit.INCH) - odomPose.getX(DistanceUnit.INCH),
                filtVision.getY(DistanceUnit.INCH) - odomPose.getY(DistanceUnit.INCH));

        if (quality >= resetMinQuality && err >= resetMinErrorInches && tagCount >= goodTagCount) {
            stableCount++;
            if (stableCount >= resetStableFrames) {
                stableCount = 0;
                didReset = true;
                // HARD reset to filtered vision pose
                fused = new Pose2D(DistanceUnit.INCH,filtVision.getX(DistanceUnit.INCH), filtVision.getY(DistanceUnit.INCH), AngleUnit.RADIANS,odomPose.getHeading(AngleUnit.RADIANS));
            }
        } else {
            stableCount = 0;
        }

        return new Output(true, quality, staleness, filtVision, fused, didReset);
    }

    // ---------- helpers ----------

    private double computeQuality(int tagCount, double tx, double ty, long stalenessMs) {
        // tagScore: 0.0 (none) -> 1.0 (>=2 tags)
        double tagScore = (tagCount >= goodTagCount) ? 1.0 : (tagCount == 1 ? 0.55 : 0.0);

        // centerScore: near center = 1, far off-center = smaller
        double txScore = 1.0 - clamp(Math.abs(tx) / maxTxForHighTrustDeg, 0.0, 1.0);
        double tyScore = 1.0 - clamp(Math.abs(ty) / maxTyForHighTrustDeg, 0.0, 1.0);
        double centerScore = 0.5 * (txScore + tyScore);

        // freshnessScore: staleness near 0 is best
        double freshnessScore = 1.0 - clamp(stalenessMs / (double) maxStalenessMs, 0.0, 1.0);

        // Weighted blend (tweak weights as needed)
        double q = 0.55 * tagScore + 0.30 * centerScore + 0.15 * freshnessScore;
        return clamp(q, 0.0, 1.0);
    }

    private Pose2D filterVision(Pose2D v, double alpha) {
        if (Double.isNaN(filtX)) {
            filtX = v.getX(DistanceUnit.INCH);
            filtY = v.getY(DistanceUnit.INCH);
            filtH = v.getHeading(AngleUnit.RADIANS); // not used in fused, but kept if you want
        } else {
            filtX = filtX + alpha * (v.getX(DistanceUnit.INCH) - filtX);
            filtY = filtY + alpha * (v.getY(DistanceUnit.INCH) - filtY);
            // optional heading filtering
            filtH = lerpAngle(filtH, v.getHeading(AngleUnit.RADIANS), alpha);
        }
        return new Pose2D(DistanceUnit.INCH,filtX, filtY, AngleUnit.RADIANS,v.getHeading(AngleUnit.RADIANS));
    }

    private Pose2D fuseXYKeepHeading(Pose2D odom, Pose2D vision, double alpha) {
        double x = odom.getX(DistanceUnit.INCH) + alpha * (vision.getX(DistanceUnit.INCH) - odom.getX(DistanceUnit.INCH));
        double y = odom.getY(DistanceUnit.INCH) + alpha * (vision.getY(DistanceUnit.INCH) - odom.getY(DistanceUnit.INCH));
        return new Pose2D(DistanceUnit.INCH,x, y, AngleUnit.RADIANS,odom.getHeading(AngleUnit.RADIANS));
    }

    private static double hypot(double a, double b) { return Math.hypot(a, b); }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double lerpAngle(double from, double to, double a) {
        double d = wrapAngle(to - from);
        return wrapAngle(from + d * a);
    }

    private static double wrapAngle(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        return rad;
    }
}

/**
 * 3) Tuning cheatsheet (so it stops being jumpy)
 *
 * If you still see noise while stationary:
 *
 * Make it smoother (less jitter)
 *
 * *Lower alphaMax (e.g. 0.18)
 *
 * *Lower alphaMin (e.g. 0.02)
 *
 * *Increase resetStableFrames (e.g. 5–6)
 *
 * Make it accept only the best frames
 *
 * *goodTagCount = 2
 *
 * *Lower maxTxForHighTrustDeg/maxTyForHighTrustDeg (e.g. 8–10)
 *
 * *Lower maxStalenessMs (e.g. 80–100)
 *
 * Prevent “teleport corrections”
 *
 * *Decrease maxStepInches (e.g. 6–8)
 */
