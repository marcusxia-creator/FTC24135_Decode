package org.firstinspires.ftc.teamcode.TeleOp;

import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;

import org.firstinspires.ftc.teamcode.TeleOp.Commands.RobotDriveCommand;
///Subsystems
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotDriveSubsystem;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

@TeleOp (name = "Basic TeleOp", group = "Test")

public class BasicTeleOp extends CommandOpMode {
    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private GamepadEx gamepad;

    @Override
    public void initialize (){
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        shooter = new ShooterSubsystem(robot);
        gamepad = new GamepadEx(gamepad1);

        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> -gamepad.getRightY(),
                () -> gamepad.getRightX(),
                () -> gamepad.getLeftX()
        ));
        ///------------------Gamepad-----------------------
        ///Run intake
        new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(intake :: runRollers,intake))
                .whenReleased(new InstantCommand(intake :: stop,intake));
        new GamepadButton (gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed (new InstantCommand(() -> intake.increasePower(), intake));
        new GamepadButton (gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(()-> intake.decreasePower(), intake));
        ///RUN SHOOTER
        new GamepadButton(gamepad, GamepadKeys.Button.X)
                .whileHeld(new RunCommand(shooter :: runFlywheel, shooter))
                .whenReleased(new InstantCommand(shooter :: stop, shooter));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand (shooter :: increasePower, shooter));
        new GamepadButton (gamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand (shooter :: decreasePower, shooter));
        ///STOP
        new GamepadButton(gamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {shooter.stop();
                    intake.stop();
                }, shooter, intake));

    }
    @Override
    public void run (){
        super.run();
        telemetry.addData("Intake Motor Power", intake.getIntakeMotorPower());
        telemetry.addData("Shooter Power",shooter.getshooterPower());
        telemetry.update();
    }
}