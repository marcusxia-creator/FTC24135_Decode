package org.firstinspires.ftc.teamcode.IceWaddler2.Teleops;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@TeleOp(name="Velocity PID Tuner", group="IceWaddler")
public class VelPIDTuner extends OpMode {
    RobotHardware robot;
    IceWaddler waddler;
    FtcDashboard dashboard;

    @Override
    public void init(){
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(new Position(new Vector(0,0,m),new NormalizedAngle(0,deg)),true);

        dashboard=FtcDashboard.getInstance();

        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop(){
        Scalar linVelFactor=new Scalar(6,metersPerSecond);
        Scalar angVelFactor=new Scalar(3,radiansPerSecond);

        telemetry.addData("Current x vel",waddler.getCurrentSituation().getVelocity().getX().getValueSI());
        telemetry.addData("Current y vel",waddler.getCurrentSituation().getVelocity().getY().getValueSI());
        telemetry.addData("Current ang vel",waddler.getCurrentSituation().getVelocity().getAngVel().getValueSI());

        telemetry.addData("Target x vel", linVelFactor.multiply(gamepad1.right_stick_x).getValueSI());
        telemetry.addData("Target y vel", linVelFactor.multiply(gamepad1.right_stick_y).getValueSI());
        telemetry.addData("Target ang vel", angVelFactor.multiply(gamepad1.left_stick_x).getValueSI());
    }
}
