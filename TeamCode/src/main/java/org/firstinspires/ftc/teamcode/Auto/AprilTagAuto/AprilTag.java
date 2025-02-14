package org.firstinspires.ftc.teamcode.Auto.AprilTagAuto;

import android.util.Size;
import java.util.HashMap; // import the HashMap class

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTag {

    private static AprilTagProcessor tagProcessor;
    private static VisionPortal visionPortal;
    private static HardwareMap hardwareMap;

    HashMap<Integer, Double []> AprilTagID = new HashMap<>();


    public AprilTag(HardwareMap hardwareMap) {
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


        visionPortal.resumeLiveView();
        visionPortal.resumeLiveView();

    }


    //public static double getPose () {
        //return
    //}
}
