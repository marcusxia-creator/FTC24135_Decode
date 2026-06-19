package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.Teleops;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Vector;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@TeleOp(name="Velocity PID Tuner", group="IceWaddler")
public class VelPIDTuner extends OpMode {
    RobotHardware robot;

    IceWaddler waddler;

    @Override
    public void init(){
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(IceWaddler.CONTROLMODE.VELOCITY, new Position(new Vector(0,0,m),new NormalizedAngle(0,deg)),true);
    }

    @Override
    public void loop(){
        Scalar linVelFactor=new Scalar(3,metersPerSecond);
        Scalar angVelFactor=new Scalar(3,radiansPerSecond);
        waddler.runByVel(
                new Velocity(new Vector(linVelFactor.multiply(gamepad1.right_stick_x),linVelFactor.multiply(gamepad1.right_stick_y)),angVelFactor.multiply(gamepad1.left_stick_x))
        );
        waddler.loop();
    }
}
