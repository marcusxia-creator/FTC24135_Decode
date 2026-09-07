package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;

import org.firstinspires.ftc.teamcode.TeleOp.Commands.LimelightCommand;
import org.firstinspires.ftc.teamcode.TeleOp.Commands.RobotDriveCommand;
///Subsystems
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.LinkagePTOSubsystem;

import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

@TeleOp (name = "Intake Test TeleOp", group = "Test")
@Config
public class IntakeTestTeleOp extends CommandOpMode {
    private RobotDriveSubsystem robotDrive;
    private LinkagePTOSubsystem linkagePTO;
    private LiftSubsystem lift;
    private IntakeSubsystem intake;
    private GamepadEx gamepad;

    private LimelightCommand limelightCommand;
    private LimelightSubsystem limelightSubsystem;
    private FtcDashboard dashboard;

    @Override
    public void initialize (){
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        gamepad = new GamepadEx(gamepad1);
        lift = new LiftSubsystem(robot);

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
        ///Run lift
        new GamepadButton(gamepad, GamepadKeys.Button.Y)
            .whenPressed(new InstantCommand (lift :: extendLift, lift));
        new GamepadButton(gamepad, GamepadKeys.Button.X)
            .whenPressed(new InstantCommand(lift :: lowerLift, intake));

        limelightSubsystem = new LimelightSubsystem(robot);
        limelightCommand = new LimelightCommand(limelightSubsystem);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }
    @Override
    public void run (){
        super.run();
        //telemetry.addData("Intake Power", intake.getIntakePower());
        telemetry.addData("tx", limelightSubsystem.getTx());
        telemetry.addData("ty", limelightSubsystem.getTy());
        telemetry.update();
    }
}