package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotHardware;

@TeleOp (name = "Intake Test TeleOp", group = "Test")

public class IntakeTestTeleOp extends OpMode {
    private RobotHardware robot;

    @Override
    public void init (){
        robot = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop (){

    }
}





