package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auto.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp(name="goBilda Odo Offset Tester")
public class OdoOffsetTester extends OpMode {
    RobotHardware robot;
    GoBildaPinpointDriver odo;
    IWDriveTrain driveTrain;

    @Override
    public void init() {
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        odo=robot.odo;
        odo.initialize();
        odo.resetPosAndIMU();

        driveTrain=robot.driveTrain;
    }

    @Override
    public void loop() {
        double power=gamepad1.left_stick_x;
        driveTrain.runPowers(power,power,-power,-power);

        odo.update();

        telemetry.addData("X Offset", String.format("%d mm", (odo.getEncoderX()/13.26291192)/odo.getHeading()));
        telemetry.addData("Y Offset", String.format("%d mm", (odo.getEncoderY()/13.26291192)/odo.getHeading()));
    }
}
