package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.TeleOps.Spindexer.Motif;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Map;

public class MotifDetector {
    Map<Integer, Motif> motifMap;
    AprilTagProcessor processor;
    VisionPortal portal;

    public MotifDetector(Map<Integer, Motif> motifMap, CameraName camera){
        this.motifMap = motifMap;
        processor = new AprilTagProcessor.Builder().build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(camera)
                .build();
        portal.setProcessorEnabled(processor,false);
    }

    public void detectMotif(){
        portal.setProcessorEnabled(processor,true);
        ArrayList<AprilTagDetection> detections = processor.getDetections();
        if(!detections.isEmpty()){
            int id=detections.get(0).id;
            if(motifMap.containsKey(id)){
                MotifMemorization.motif=motifMap.get(id);
            }
        }
    }

    public static class MotifMemorization {
        public static Motif motif;
    }
}
