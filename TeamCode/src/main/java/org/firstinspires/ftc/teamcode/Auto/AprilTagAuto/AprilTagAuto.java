package org.firstinspires.ftc.teamcode.Auto.AprilTagAuto;

import android.util.Size;

import androidx.annotation.NonNull;

import java.util.HashMap; // import the HashMap class

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagAuto {

    private static AprilTagProcessor tagProcessor;
    private static VisionPortal visionPortal;
    private static HardwareMap hardwareMap;

    public static Integer[] aprilTagCoordinateArray = {null, null};
    public static Long[] tagInfo = {null, null, null, null};
    private static HashMap <String, Double> robotFieldCoordinate = new HashMap<>();
    private static int tagID = 0;

    public AprilTagAuto(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init () {
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


        visionPortal.resumeStreaming();

    }

    @NonNull
    public static HashMap<Integer, Integer[]> aprilTagMsg() {

        HashMap<Integer, Integer[]> aprilTagCoordinate = new HashMap<>();
        Integer[] aprilTagCoordinateArrayID11 = new Integer[]{-72, 48};
        Integer[] aprilTagCoordinateArrayID12 = new Integer[]{  0, 72};
        Integer[] aprilTagCoordinateArrayID13 = new Integer[]{ 72, 48};
        Integer[] aprilTagCoordinateArrayID14 = new Integer[]{ 72,-48};
        Integer[] aprilTagCoordinateArrayID15 = new Integer[]{ 0 ,-72};
        Integer[] aprilTagCoordinateArrayID16 = new Integer[]{-72,-48};

        aprilTagCoordinate.put(11, aprilTagCoordinateArrayID11);
        aprilTagCoordinate.put(12, aprilTagCoordinateArrayID12);
        aprilTagCoordinate.put(13, aprilTagCoordinateArrayID13);
        aprilTagCoordinate.put(14, aprilTagCoordinateArrayID14);
        aprilTagCoordinate.put(15, aprilTagCoordinateArrayID15);
        aprilTagCoordinate.put(16, aprilTagCoordinateArrayID16);

        return aprilTagCoordinate;
    }

    @NonNull
    public static Double[] robotOffSet() {

        /** This is the test bot config, the actual bot is different **/

        return new Double[]{-4.38, 3.25};
    }

    public Integer[] aprilTagUpdate() {

        if (!tagProcessor.getDetections().isEmpty()) {

            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            aprilTagCoordinateArray = aprilTagMsg().get(tag.id);

        }
        return aprilTagCoordinateArray;
    }

    public Long[] tagAxis() {

        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            tagInfo[0] = Math.round (tag.ftcPose.x * 100) /100;
            tagInfo[1] = Math.round (tag.ftcPose.y * 100) /100;
            tagInfo[2] = Math.round (tag.ftcPose.bearing * 100) /100;
            tagInfo[3] = Math.round (tag.ftcPose.yaw * 100) /100;
        }
        return tagInfo;
    }

    public int tagID () {

        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            tagID = tag.id;
        }
        return tagID;
    }

    public HashMap<String, Double> robotFieldCoordinate() {

        aprilTagUpdate();
        tagAxis();

        Integer[] aprilTagCoordinate = {aprilTagUpdate()[0], aprilTagUpdate()[1]};
        Double[] robotValueOffSet = {robotOffSet()[0], robotOffSet()[1]};
        Long[] robotCoordinate = {tagAxis()[0],tagAxis()[1]};

        if ((aprilTagCoordinateArray[0] != null && aprilTagCoordinateArray[1] != null) && (tagInfo[0] != null && tagInfo[1] != null)) {
            robotFieldCoordinate.put("x", (robotCoordinate[0] + robotValueOffSet[0]) + aprilTagCoordinate[0]);
            robotFieldCoordinate.put("y", (robotCoordinate[1] + robotValueOffSet[1]) + aprilTagCoordinate[1]);
        }

        return robotFieldCoordinate;
    }

}
