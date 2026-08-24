package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.GamepadDriver;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotDrive;

import CommandBase.Action;
import CommandBase.PrebuiltActions.*;

@TeleOp(name="Sample Command Based OpMode")
@Disabled
public class SchedulerTest extends OpMode {
    Action rootAction; //Set this to desired action

    RobotHardware robot;

    GamepadDriver gamepad;

    RobotDrive drive;
    Intake intake;

    @Override
    public void init(){
        robot=new RobotHardware(hardwareMap);
        gamepad=new GamepadDriver(gamepad1,gamepad2);

        drive=new RobotDrive(robot,gamepad::driveX,gamepad::driveY,gamepad::driveRot);
        intake=new Intake(robot);

        rootAction=new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,
                drive.manualDrive,
                new ActivatableAction(gamepad::intakeStart, gamepad::cancel, intake.runIntake)
        );
    }

    @Override
    public void start() {
        rootAction.init();
    }

    @Override
    public void loop(){
        rootAction.loop();
    }

    @Override
    public void stop(){
        rootAction.shutdown();
    }
}
