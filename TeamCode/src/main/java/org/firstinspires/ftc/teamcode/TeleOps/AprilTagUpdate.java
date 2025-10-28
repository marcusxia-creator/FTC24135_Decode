package org.firstinspires.ftc.teamcode.TeleOps;

import android.util.Size;
import androidx.annotation.NonNull;
import java.util.HashMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagUpdate {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;

    // Array to hold the detected AprilTag coordinate offset
    private Integer[] aprilTagCoordinateArray = {null, null};

    // Array to hold tag axis information as doubles for precision
    private double[] tagInfo = {0.0, 0.0, 0.0, 0.0};

    /** This is the test bot config, the actual bot is different **/
    private Vector2D robotOffset = new Vector2D(-4.38, 3.25); // Initial value

    // Holds the calculated robot field coordinates
    private HashMap<String, Double> robotFieldCoordinate = new HashMap<>();
    private int tagID = 0;

    public AprilTagUpdate(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Web_Cam"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    /**
     * Returns a mapping from AprilTag IDs to their pre-defined field coordinates.
     */
    @NonNull
    public HashMap<Integer, Integer[]> getAprilTagCoordinates() {
        HashMap<Integer, Integer[]> aprilTagCoordinates = new HashMap<>();
        aprilTagCoordinates.put(11, new Integer[]{72, 72});
        aprilTagCoordinates.put(16, new Integer[]{-72, -48});
        return aprilTagCoordinates;
    }

    /**
     * Returns the current robot offset.
     */
    public Vector2D getRobotOffset() {
        return robotOffset;
    }

    /**
     * Updates the AprilTag coordinate array based on the first detected tag.
     */
    public void updateAprilTag() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            Integer[] coordinates = getAprilTagCoordinates().get(tag.id);
            if (coordinates != null) {
                aprilTagCoordinateArray = coordinates;
            }else{
                aprilTagCoordinateArray = new Integer[]{null, null};
            }
        }
    }

    /**
     * Updates the tag axis information (position and orientation) with rounding to 2 decimals.
     */
    public void updateTagAxis() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            tagInfo[0] = Math.round(tag.ftcPose.x * 100.0) / 100.0;
            tagInfo[1] = Math.round(tag.ftcPose.y * 100.0) / 100.0;
            tagInfo[2] = Math.round(tag.ftcPose.bearing * 100.0) / 100.0;
            tagInfo[3] = Math.round(tag.ftcPose.yaw * 100.0) / 100.0;
        }
    }

    /**
     * Returns the ID of the first detected tag.
     */
    public int getTagID() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            tagID = tag.id;
        }
        return tagID;
    }

    /**
     * Calculates and returns the robot's field coordinate based on tag data, robot offset, and pre-defined tag positions.
     */
    public HashMap<String, Double> calculateRobotFieldCoordinate() {
        updateAprilTag();
        updateTagAxis();

        // Clear previous field coordinates to update dynamically
        robotFieldCoordinate.clear();

        if (aprilTagCoordinateArray[0] != null && aprilTagCoordinateArray[1] != null) {
            double robotX = tagInfo[0];
            double robotY = tagInfo[1];
            double offsetX = getRobotOffset().getX();
            double offsetY = getRobotOffset().getY();
            // Calculate field coordinate by summing the robot's pose (with offset) and the tag's field coordinate
            robotFieldCoordinate.put("x", robotX + offsetX + aprilTagCoordinateArray[0]);
            robotFieldCoordinate.put("y", robotY + offsetY + aprilTagCoordinateArray[1]);
        }
        return robotFieldCoordinate;
    }
}
