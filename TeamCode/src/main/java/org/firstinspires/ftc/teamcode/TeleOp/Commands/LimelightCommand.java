package org.firstinspires.ftc.teamcode.TeleOp.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.LimelightSubsystem;

public class LimelightCommand extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;

    public LimelightCommand(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;

        addRequirements(limelightSubsystem);
    }

    @Override
    public void execute() {
        limelightSubsystem.run();
    }

}
