package org.firstinspires.ftc.teamcode.TeleOps.Sensors;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class FSMAprilTagProc {
    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;
    private FtcDashboard ftcDashboard;

    private RobotHardware robot;

    public AprilTagDetection tag;

    public double Heading;
    public double Distance;

    public enum ProcState{
        RUNNING,
        STOPPED
    }

    public ProcState procState;

    public FSMAprilTagProc(RobotHardware robot){
        this.robot = robot;
    }

    public void init () {

        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                //.setCamera(robot.webcam1)
                .setCameraResolution(new Size(640, 480))
                .build();

        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();

        procState = ProcState.STOPPED;
    }

    public void loop (){
        switch(procState) {
            case STOPPED:
                break;

            case RUNNING:
                tag = tagProcessor.getDetections().get(0);
                Heading = tag.center.x;
                Distance = Math.sqrt(Math.pow(tag.ftcPose.x, 2) + Math.pow(tag.ftcPose.y, 2));
                break;
        }
    }
}
