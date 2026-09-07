package org.firstinspires.ftc.teamcode.IceWaddler2.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.DimlessVector;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

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
        double voltage=robot.voltageSensor.getVoltage();
        double forward=-gamepad1.right_stick_y*voltage;
        double strafe=gamepad1.right_stick_x*voltage;
        double rot=gamepad1.left_stick_x*voltage;

        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Rot", rot);
        telemetry.update();

        driveTrain.run(new DimlessVector(strafe,forward),rot);
    }
}
