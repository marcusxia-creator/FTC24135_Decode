package org.firstinspires.ftc.teamcode.TeleOps.Sensors;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayDeque;

public class VisionResidualStdDev {
    public static class Stats {
        public final int n;
        public final double meanRxMm, meanRyMm; // bias (mm) - mean of ox-vx residual, mean of oy-vy residual
        public final double stdRxMm, stdRyMm;   // jitter (mm) - standard deviation of ox-vx residual, standard deviation of oy-vy residual
        public final double stdRmm;             // combined jitter (mm) - combined standard deviation
        public final boolean valid;

        public Stats(int n, double meanRxMm, double meanRyMm, double stdRxMm, double stdRyMm, boolean valid) {
            this.n = n;
            this.meanRxMm = meanRxMm;
            this.meanRyMm = meanRyMm;
            this.stdRxMm = stdRxMm;
            this.stdRyMm = stdRyMm;
            this.stdRmm = Math.hypot(stdRxMm, stdRyMm);
            this.valid = valid;
        }
    }

    /**
     * meanRxMm / meanRyMm = bias (systematic offset between Limelight and Pinpoint)
     * stdRmm = noise/jitter of vision relative to odom (this is what you want for “good/bad frame”)
     */

    public int windowSize = 25;
    public int minSamples = 10;

    private final ArrayDeque<double[]> buf = new ArrayDeque<>();

    public void clear() { buf.clear(); }

    public Stats updateAndGet(Pose2D odom, Pose2D vision) {
        double ox = odom.getX(DistanceUnit.MM);
        double oy = odom.getY(DistanceUnit.MM);

        double vx = vision.getX(DistanceUnit.MM);
        double vy = vision.getY(DistanceUnit.MM);

        double rx = vx - ox;
        double ry = vy - oy;

        buf.addLast(new double[]{rx, ry});
        while (buf.size() > windowSize) buf.removeFirst();

        return compute();
    }

    private Stats compute() {



        int n = buf.size();
        if (n < minSamples) return new Stats(n, 0, 0, Double.NaN, Double.NaN, false);

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

        return new Stats(n, meanX, meanY, Math.sqrt(varX), Math.sqrt(varY), true);
    }
}
