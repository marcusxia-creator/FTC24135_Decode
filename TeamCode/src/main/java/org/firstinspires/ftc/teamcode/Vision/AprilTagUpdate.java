package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;
import androidx.annotation.NonNull;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOps.BallColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class AprilTagUpdate {

    private final HardwareMap hardwareMap;
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    // --- Detection & Pose Data ---
    private int tagID = -1;
    private final double[] tagInfo = {0.0, 0.0, 0.0, 0.0};  // x, y, bearing, yaw
    private Integer[] aprilTagCoordinateArray = {null, null};
    private final Vector2D robotOffset = new Vector2D(-4.38, 3.25);
    private final HashMap<String, Double> robotFieldCoordinate = new HashMap<>();

    // --- Color Sequence Data ---
    private final Map<Integer, BallColor[]> aprilTagSequences = new HashMap<>();
    private BallColor[] selectedSequence = new BallColor[0];

    public AprilTagUpdate(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        // Initialize AprilTag processor & camera
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        // Preload lookup tables
        initAprilTagSequences();
    }

    // ============================================================
    // === Initialization Helpers ===
    // ============================================================

    /** Map of AprilTag IDs to their field coordinates */
    @NonNull
    public Map<Integer, Integer[]> getAprilTagCoordinates() {
        Map<Integer, Integer[]> aprilTagCoordinates = new HashMap<>();
        aprilTagCoordinates.put(11, new Integer[]{72, 72});
        aprilTagCoordinates.put(16, new Integer[]{-72, -48});
        return aprilTagCoordinates;
    }

    /** Predefined color sequences per tag */
    private void initAprilTagSequences() {
        aprilTagSequences.put(21, new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE});
        aprilTagSequences.put(22, new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE});
        aprilTagSequences.put(23, new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN});
    }

    // ============================================================
    // === Detection Methods ===
    // ============================================================

    /** Updates the tag ID and pose information. Call in INIT-loop. */
    public void update() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            tagID = tag.id;

            // --- Update pose info ---
            if (tag.id == 20 || tag.id == 24) {
                tagInfo[0] = round2(tag.ftcPose.x);
                tagInfo[1] = round2(tag.ftcPose.y);
                tagInfo[2] = round2(tag.ftcPose.bearing);
                tagInfo[3] = round2(tag.ftcPose.yaw);

                // --- Lookup coordinates if known ---
                Integer[] coords = getAprilTagCoordinates().get(tag.id);
                aprilTagCoordinateArray = (coords != null) ? coords : new Integer[]{null, null};
            } else {
                // For ignored tags, reset to neutral / safe state
                tagInfo[0] = tagInfo[1] = tagInfo[2] = tagInfo[3] = 0.0;
                aprilTagCoordinateArray = new Integer[]{null, null};
            }

            // --- Lookup color sequence ---
            if (tag.id == 21 || tag.id == 22 || tag.id == 23) {
                selectedSequence = aprilTagSequences.getOrDefault(
                        tag.id,
                        new BallColor[]{BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN}
                );
            } else {
                tagID = -1;
                selectedSequence = new BallColor[]{BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN};
            }
        }
    }

    // ============================================================
    // === Getters ===
    // ============================================================

    public int getTagID() {
        return tagID;
    }

    public BallColor[] getSequence() {
        return selectedSequence;
    }

    public String getSequenceAsString() {
        return Arrays.toString(selectedSequence);
    }

    public double[] getTagInfo() {
        return tagInfo;
    }

    public Vector2D getRobotOffset() {
        return robotOffset;
    }

    public Integer[] getTagCoordinate() {
        return aprilTagCoordinateArray;
    }

    // ============================================================
    // === Field Localization ===
    // ============================================================

    /**
     * Calculates robot field coordinates based on tag pose, offset, and known tag positions.
     */
    public Map<String, Double> calculateRobotFieldCoordinate() {
        robotFieldCoordinate.clear();

        if (aprilTagCoordinateArray[0] != null && aprilTagCoordinateArray[1] != null) {
            double robotX = tagInfo[0];
            double robotY = tagInfo[1];
            double offsetX = robotOffset.getX();
            double offsetY = robotOffset.getY();

            robotFieldCoordinate.put("x", robotX + offsetX + aprilTagCoordinateArray[0]);
            robotFieldCoordinate.put("y", robotY + offsetY + aprilTagCoordinateArray[1]);
        }

        return robotFieldCoordinate;
    }

    public String getRobotFieldCoordinateAsString() {
        if (robotFieldCoordinate.isEmpty()) return "N/A";
        return String.format("(%.2f, %.2f)",
                robotFieldCoordinate.getOrDefault("x", 0.0),
                robotFieldCoordinate.getOrDefault("y", 0.0)
        );
    }

    // ============================================================
    // === Utility ===
    // ============================================================

    private double round2(double v) {
        return Math.round(v * 100.0) / 100.0;
    }

    /** Optional: return tag info summary for telemetry */
    public String getTagPoseSummary() {
        return String.format("x: %.2f, y: %.2f, bearing: %.2f, yaw: %.2f",
                tagInfo[0], tagInfo[1], tagInfo[2], tagInfo[3]);
    }
}
