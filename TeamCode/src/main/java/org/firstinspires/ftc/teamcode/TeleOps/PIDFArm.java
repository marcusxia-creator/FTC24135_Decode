package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled

@Config
@TeleOp (name = "PIDF Arm", group = "org.firstinspires.ftc.teamcode")
public class PIDFArm extends OpMode{
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    public final double ticksPerDegree = 384.5/360;

    RobotHardware robot;
    @Override
    public void init (){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop (){
        controller.setPID(p,i,d);
        int armPos = robot.liftMotorLeft.getCurrentPosition();
        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target / ticksPerDegree)) * f;

        double power = pid + ff;

        robot.liftMotorLeft.setPower(power);
        robot.liftMotorRight.setPower(power);

        telemetry.addData("position", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

}
