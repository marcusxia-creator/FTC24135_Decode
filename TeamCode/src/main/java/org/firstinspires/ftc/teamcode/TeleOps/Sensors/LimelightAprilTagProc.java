package org.firstinspires.ftc.teamcode.TeleOps.Sensors;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import java.util.List;
import java.util.Optional;

public class LimelightAprilTagProc {
    public enum ProcState{
        RUNNING,
        STOPPED
    }

    private FtcDashboard ftcDashboard;
    private RobotHardware robot;
    Limelight3A limelight;
    public ProcState procState;
    private int aprilTagID;;

    public LimelightAprilTagProc(RobotHardware robot){
        this.robot = robot;
    }


    public void init () {
        limelight = robot.limelight3A;
        procState = ProcState.STOPPED;
    }

    public void loop (){

        LLResult r = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = r.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            aprilTagID = fiducial.getFiducialId(); // The ID number of the fiducial;
        }
        telemetry.addData("AprilTagID", aprilTagID);
    }
    public void stop(){
        procState = ProcState.STOPPED;
    }
    public Optional<Integer> getAprilTagID() {
        return Optional.ofNullable(aprilTagID);
    }

}
