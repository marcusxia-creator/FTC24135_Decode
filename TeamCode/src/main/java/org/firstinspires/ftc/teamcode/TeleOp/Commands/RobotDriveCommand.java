package org.firstinspires.ftc.teamcode.TeleOp.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.RobotHardware;

import java.util.function.DoubleSupplier;

public class RobotDriveCommand extends CommandBase {
    private final RobotDriveSubsystem robotDrive;

    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;

    public RobotDriveCommand(RobotDriveSubsystem robotDrive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.robotDrive = robotDrive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;

        addRequirements(robotDrive);
    }
    @Override
    public void execute() {
        robotDrive.mecanumDrive(
                forward.getAsDouble(),
                strafe.getAsDouble(),
                turn.getAsDouble());
    }

}

