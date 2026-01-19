package org.firstinspires.ftc.teamcode.TeleOps.Sensors;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class VisionResetManager {

    // ---- Tunables ----
    public long maxStalenessMs = 120;

    public int minTagsForReset = 2;
    public double maxAbsTxDeg = 12;
    public double maxAbsTyDeg = 12;

    public int stableFramesNeeded = 5;
    public long resetCooldownMs = 1200;

    public double residualToRequestResetMm = 350; // if vision disagrees this much, request reset
    public double stdRMaxForResetMm = 120;        // residual jitter must be under this

    // Optional: Only reset if Pinpoint likely got disturbed
    public boolean requireNeedsResetFlag = false;

    // ---- State ----
    private final VisionResidualStdDev residualStd = new VisionResidualStdDev();
    private int stableCount = 0;
    private long lastResetTimeMs = 0;
    private boolean needsReset = false;

    private static final double MM_PER_M = 1000.0;

    public static class Decision {
        public final boolean hasVision;
        public final boolean shouldReset;
        public final Pose2D visionPose;
        public final VisionResidualStdDev.Stats residualStats;

        public Decision(boolean hasVision, boolean shouldReset, Pose2D visionPose, VisionResidualStdDev.Stats stats) {
            this.hasVision = hasVision;
            this.shouldReset = shouldReset;
            this.visionPose = visionPose;
            this.residualStats = stats;
        }
    }

    /** Call this if you detect a collision/slip/etc. */
    public void markNeedsReset() {
        needsReset = true;
    }

    /** Call after a successful reset. */
    private void clearNeedsReset() {
        needsReset = false;
    }

    /**
     * Main decision function.
     * @param limelight Limelight3A
     * @param imuYawDeg Your IMU yaw (deg) used for heading
     * @param odomPose  Pinpoint pose (FTC field) in MM/DEG
     * @param nowMs     current time in ms (e.g., (long)(getRuntime()*1000))
     */
    public Decision update(Limelight3A limelight, double imuYawDeg, Pose2D odomPose, long nowMs) {

        // Cooldown check
        if (nowMs - lastResetTimeMs < resetCooldownMs) {
            stableCount = 0;
        }

        limelight.updateRobotOrientation(imuYawDeg);

        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid() || r.getStaleness() > maxStalenessMs) {
            stableCount = 0;
            return new Decision(false, false, null, null);
        }

        Pose3D p = r.getBotpose_MT2();
        if (p == null) {
            stableCount = 0;
            return new Decision(false, false, null, null);
        }

        int tagCount = (r.getFiducialResults() == null) ? 0 : r.getFiducialResults().size();
        double tx = r.getTx();
        double ty = r.getTy();

        // Vision pose (meters -> mm), heading from IMU
        double xMm = p.getPosition().x * MM_PER_M;
        double yMm = p.getPosition().y * MM_PER_M;

        Pose2D visionPose = new Pose2D(DistanceUnit.MM, xMm, yMm, AngleUnit.DEGREES, imuYawDeg);

        // Residual stats (works while moving)
        VisionResidualStdDev.Stats st = residualStd.updateAndGet(odomPose, visionPose);

        // If vision strongly disagrees with odom, request reset
        double ox = odomPose.getX(DistanceUnit.MM);
        double oy = odomPose.getY(DistanceUnit.MM);
        double residualMm = Math.hypot(xMm - ox, yMm - oy);
        if (residualMm > residualToRequestResetMm) {
            needsReset = true;
        }

        boolean gates =
                tagCount >= minTagsForReset &&
                        Math.abs(tx) <= maxAbsTxDeg &&
                        Math.abs(ty) <= maxAbsTyDeg &&
                        st != null && st.valid &&
                        st.stdRmm <= stdRMaxForResetMm &&
                        (nowMs - lastResetTimeMs >= resetCooldownMs);

        if (gates && (!requireNeedsResetFlag || needsReset)) {
            stableCount++;
        } else {
            stableCount = 0;
        }

        boolean shouldReset = stableCount >= stableFramesNeeded;

        if (shouldReset) {
            stableCount = 0;
            lastResetTimeMs = nowMs;
            clearNeedsReset();
        }

        return new Decision(true, shouldReset, visionPose, st);
    }
}
