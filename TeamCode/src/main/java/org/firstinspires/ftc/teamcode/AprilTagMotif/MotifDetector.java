package org.firstinspires.ftc.teamcode.AprilTagMotif;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.Spindexer.Motif;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Map;

public class MotifDetector {
    //Apriltag ID and sequence map
    Map<Integer, Motif> motifMap;
    AprilTagProcessor processor;
    VisionPortal portal;

    public MotifDetector(Map<Integer, Motif> motifMap, RobotHardware robot){
        this.motifMap = motifMap;
        processor = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .build();
        portal.setProcessorEnabled(processor,true);
    }

    public void detectMotif(){
        ArrayList<AprilTagDetection> detections = processor.getDetections();
        if(!detections.isEmpty()){
            int id=detections.get(0).id;
            if(motifMap.containsKey(id)){
                MotifMemorization.motif=motifMap.get(id);
            }
        }
        /**
        if(MotifMemorization.motif!=null){
            portal.setProcessorEnabled(processor,false);
            portal = null;
        }
         */
    }
}
