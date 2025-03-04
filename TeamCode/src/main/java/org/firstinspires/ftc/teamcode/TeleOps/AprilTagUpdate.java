package org.firstinspires.ftc.teamcode.TeleOps;

import android.util.Size;

import androidx.annotation.NonNull;

import java.util.HashMap; // import the HashMap class
import java.util.Vector;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagUpdate {

    private static AprilTagProcessor tagProcessor;
    private static VisionPortal visionPortal;
    private static HardwareMap hardwareMap;

    private static Integer[] aprilTagCoordinateArray = {null, null};
    private static Long[] tagInfo = {null, null, null, null};
    /** This is the test bot config, the actual bot is different **/
    private static Vector2D robotOffSet = new Vector2D(-4.38, 3.25); //Initial value
    private static HashMap <String, Double> robotFieldCoordinate = new HashMap<>();
    private static int tagID = 0;

    public AprilTagUpdate(HardwareMap hardwareMap) {
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
    public static Vector2D robotOffSet(Vector2D robotOffSet) {

        return robotOffSet;
    }

    public void aprilTagUpdate() {

        if (!tagProcessor.getDetections().isEmpty()) {

            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            aprilTagCoordinateArray = aprilTagMsg().get(tag.id);

        }
    }

    public void tagAxisUpdate() {

        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            tagInfo[0] = Math.round (tag.ftcPose.x * 100) /100;
            tagInfo[1] = Math.round (tag.ftcPose.y * 100) /100;
            tagInfo[2] = Math.round (tag.ftcPose.bearing * 100) /100;
            tagInfo[3] = Math.round (tag.ftcPose.yaw * 100) /100;
        }
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
        tagAxisUpdate();

        Integer[] aprilTagCoordinate = {aprilTagCoordinateArray[0], aprilTagCoordinateArray[1]};
        Vector2D offset = robotOffSet(robotOffSet);
        Long[] robotCoordinate = {tagInfo[0],tagInfo[1]};

        if ((aprilTagCoordinateArray[0] != null && aprilTagCoordinateArray[1] != null) && (tagInfo[0] != null && tagInfo[1] != null)) {
            robotFieldCoordinate.put("x", (robotCoordinate[0] + offset.getX()) + aprilTagCoordinate[0]);
            robotFieldCoordinate.put("y", (robotCoordinate[1] + offset.getY()) + aprilTagCoordinate[1]);
        }

        return robotFieldCoordinate;
    }

}
