package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayDeque;

@Config
public class VisionProcessing {

    /**
     * meanRxMm / meanRyMm = bias (systematic offset between Limelight and Pinpoint)
     * stdRmm = noise/jitter of vision relative to odom (this is what you want for “good/bad frame”)
     */

    public int maxSamples = 100;
    public int minSamples = 50;

    private final ArrayDeque<double[]> buf = new ArrayDeque<>();

    public static double stdMultiplier = 0.8;

    public void clear() { buf.clear(); }

    public Stats updateAndGet(Pose2D odometry, Pose2D vision) {
        ///double odometryX = odometry.getX(DistanceUnit.MM);
        ///double odometryY = odometry.getY(DistanceUnit.MM);

        double visionX = vision.getX(DistanceUnit.MM);
        double visionY = vision.getY(DistanceUnit.MM);

        /**
        double visionPinpointDx = visionX - odometryX;
        double visionPinpointDy = visionY - odometryY;

        boolean pass =
                Math.abs(deltaX - stats.meanRxMm) <= stdMultiplier * stats.stdRxMm &&
                Math.abs(deltaY - stats.meanRyMm) <= stdMultiplier * stats.stdRyMm;
         */

        //Compute the existing values
        Stats previousStats = compute();

        if (!previousStats.valid) {
            buf.addLast(new double[]{visionX, visionY});
            if (buf.size() > maxSamples) buf.removeFirst();
            return compute();
        }

        /// Vision pose itself over mean ± 0.82σ (σ-std value)
        boolean outlier = true;

        double deltaX = Math.abs(visionX - previousStats.stdRxMm);
        double deltaY = Math.abs(visionY - previousStats.stdRyMm);

        double thresholdX = Math.abs(previousStats.meanRxMm * stdMultiplier);
        double thresholdY = Math.abs(previousStats.meanRyMm * stdMultiplier);

        // Prevent the threshold from collapsing at startup
        if (Double.isNaN(thresholdX) || thresholdX < 5.0) thresholdX = 5.0;
        if (Double.isNaN(thresholdY) || thresholdY < 5.0) thresholdY = 5.0;

        if (deltaX > thresholdX || deltaY > thresholdY) {
            outlier = false;
        }

        double useX = outlier ? previousStats.meanRxMm : visionX;
        double useY = outlier ? previousStats.meanRyMm : visionY;

        //buf.addLast(new double[]{deltaX, deltaY});
        buf.addLast(new double[]{useX, useY});
        while (buf.size() > maxSamples) buf.removeFirst();

        Stats stats = compute();

        if (!outlier) {
            return new Stats(stats.n, stats.meanRxMm, stats.meanRyMm, stats.stdRxMm, stats.stdRyMm, false, true);
        }

        return stats;
    }

    private Stats compute() {

        int n = buf.size();
        if (n < minSamples) return new Stats(n, 0, 0, Double.NaN, Double.NaN, false, false);

        double sumX = 0, sumY = 0;
        for (double[] r : buf) { sumX += r[0]; sumY += r[1]; }
        double meanX = sumX / n;
        double meanY = sumY / n;

        double sxx = 0, syy = 0;
        for (double[] r : buf) {
            double dx = r[0] - meanX;
            double dy = r[1] - meanY;
            sxx += dx * dx;
            syy += dy * dy;
        }

        double varX = sxx / (n - 1);
        double varY = syy / (n - 1);

        return new Stats(n, meanX, meanY, Math.sqrt(varX), Math.sqrt(varY), true, false);
    }

    ///Data Structure

    public static class Stats {
        public int n;
        public double meanRxMm, meanRyMm; // bias (mm) - mean of ox-vx residual, mean of oy-vy residual
        public double stdRxMm, stdRyMm;   // jitter (mm) - standard deviation of ox-vx residual, standard deviation of oy-vy residual
        public double stdRmm;             // combined jitter (mm) - combined standard deviation
        public boolean valid;
        public boolean filtered;


        public Stats(int n,
                     double meanRxMm, double meanRyMm,
                     double stdRxMm, double stdRyMm,
                     boolean valid, boolean filtered) {
            this.n = n;
            this.meanRxMm = meanRxMm;
            this.meanRyMm = meanRyMm;
            this.stdRxMm = stdRxMm;
            this.stdRyMm = stdRyMm;
            this.stdRmm = Math.hypot(stdRxMm, stdRyMm);
            this.valid = valid;
            this.filtered = filtered;
        }
    }
}
