package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@TeleOp (name = "VelTuner", group = "org.firstinspires.ftc.teamcode")
@Config
@Disabled
public class VelTuner extends OpMode {
    private RobotHardware robot;
    public static double power=0.7;

    public void init(){
        robot=new RobotHardware(hardwareMap);
        robot.init();
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        robot.shooterMotor.setPower(power);
        telemetry.addData("Vel", robot.shooterMotor.getVelocity());
    }
}
