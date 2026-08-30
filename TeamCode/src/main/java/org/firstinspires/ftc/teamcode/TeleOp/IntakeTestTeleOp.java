package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;

import org.firstinspires.ftc.teamcode.TeleOp.Commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotHardware;

@TeleOp (name = "Intake Test TeleOp", group = "Test")

public class IntakeTestTeleOp extends CommandOpMode {

    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private GamepadEx gamepad;

    @Override
    public void initialize (){
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        gamepad = new GamepadEx(gamepad1);

        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> -gamepad.getRightY(),
                () -> gamepad.getRightX(),
                () -> gamepad.getLeftX()
        ));

        new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(intake :: runRollers,intake))
                .whenReleased(new InstantCommand(intake :: stop,intake));
        new GamepadButton (gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed (new InstantCommand(() -> intake.increasePower(), intake));
        new GamepadButton (gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(()-> intake.decreasePower(), intake));
    }
    @Override
    public void run (){
        super.run();
        telemetry.addData("Intake Power", intake.getIntakePower());
        telemetry.update();
    }
}