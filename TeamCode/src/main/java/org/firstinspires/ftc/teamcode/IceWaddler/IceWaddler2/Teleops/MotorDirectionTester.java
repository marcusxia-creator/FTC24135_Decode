package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@TeleOp(name="Motor Direction Tester", group="IceWaddler")
public class MotorDirectionTester extends OpMode {
    RobotHardware robot;
    IWDriveTrain driveTrain;

    @Override
    public void init() {
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);
        driveTrain=robot.driveTrain;
        driveTrain.init();
    }

    @Override
    public void loop(){
        driveTrain.writePowers(0.3,0.3,0.3,0.3);
    }
}
