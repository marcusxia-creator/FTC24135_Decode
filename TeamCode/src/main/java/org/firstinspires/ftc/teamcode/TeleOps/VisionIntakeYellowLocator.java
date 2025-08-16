package org.firstinspires.ftc.teamcode.TeleOps;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

@Config
public class VisionIntakeYellowLocator {
    public static int minArea = 80;                 // pixels
    public static int maxArea = 20000;              // pixels
    public static double minAspect = 0.6;           // height/width ratio bounds (boxFit long/short)
    public static double maxAspect = 3.0;
    public static int blurSize = 5;                 // odd only; 5 works well for 320x240

    public static double roiXMin = -0.6;            // normalized unity-center ROI for performance
    public static double roiYMax = 0.6;             // search lower-middle area
    public static double roiXMax = 0.6;
    public static double roiYMin = -0.2;

    public static int cameraWidth = 320;
    public static int cameraHeight = 240;

    private VisionPortal portal;
    private ColorBlobLocatorProcessor colorLocator;

    public static class Detection {
        public boolean hasTarget;
        public int area;
        public double centerX;
        public double centerY;
        public double boxWidth;
        public double boxHeight;
        public RotatedRect box;
    }

    public void init(HardwareMap hardwareMap) {
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(roiXMin, roiYMax, roiXMax, roiYMin))
                .setDrawContours(true)
                .setBlurSize(blurSize)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    public Detection getBestDetection() {
        Detection result = new Detection();
        if (colorLocator == null) {
            result.hasTarget = false;
            return result;
        }

        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(minArea, maxArea, blobs);
        ColorBlobLocatorProcessor.Util.filterByAspectRatio(minAspect, maxAspect, blobs);
        // Default sort is by area DESC, so the first blob is the largest

        if (blobs.isEmpty()) {
            result.hasTarget = false;
            return result;
        }

        ColorBlobLocatorProcessor.Blob b = blobs.get(0);
        RotatedRect rect = b.getBoxFit();
        result.hasTarget = true;
        result.area = b.getContourArea();
        result.centerX = rect.center.x;
        result.centerY = rect.center.y;
        result.boxWidth = Math.max(rect.size.width, rect.size.height);
        result.boxHeight = Math.min(rect.size.width, rect.size.height);
        result.box = rect;
        return result;
    }

    public void close() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
        colorLocator = null;
    }
}