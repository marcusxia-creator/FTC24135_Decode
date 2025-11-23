package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.GPPid;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.PGPid;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.PPGid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.MotifDetector;
import org.firstinspires.ftc.teamcode.MotifMemorization;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.Spindexer;
import org.firstinspires.ftc.teamcode.TeleOps.Spindexer.Motif;

import java.util.Map;

@TeleOp (name = "Motif Reset", group = "org.firstinspires.ftc.teamcode")
@Config
public class MotifReset extends OpMode {
    CameraName camera;
    MotifDetector motifDetector;

    public void init(){
    }
    public void start(){
        MotifMemorization.motif=null;
    }
    public void loop(){}
}
