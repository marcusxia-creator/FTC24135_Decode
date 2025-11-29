package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.GPPid;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.PGPid;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.PPGid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.AprilTagMotif.MotifDetector;
import org.firstinspires.ftc.teamcode.AprilTagMotif.MotifMemorization;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.Spindexer.Motif;

import java.util.Map;

@TeleOp (name = "AprilTag Test", group = "org.firstinspires.ftc.teamcode")
@Config
public class AprilTagTester extends OpMode {
    MotifDetector motifDetector;
    RobotHardware robot;

    public void init(){
        robot=new RobotHardware(hardwareMap);
        robot.init();
        motifDetector = new MotifDetector(Map.of(GPPid, Motif.GPP, PGPid, Motif.PGP, PPGid, Motif.PPG),robot);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void start(){
        motifDetector.detectMotif();
        telemetry.addData("Motif", MotifMemorization.motif.name);
        telemetry.update();
    }
    public void loop(){}
}
