package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Reset Turret", group = "org.firstinspires.ftc.teamcode")
public class ResetTurret extends OpMode{
    RobotHardware robot;

    @Override
    public void init() {
        robot=new RobotHardware(hardwareMap);
        robot.init();
        robot.turretInit();

        telemetry.addLine("Turret encoder set to zero");
        telemetry.update();
    }

    public void loop(){}
}
