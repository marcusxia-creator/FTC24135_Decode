package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActionParallel.TERMINATIONTYPE.NONE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.*;
import org.firstinspires.ftc.teamcode.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.linearHP;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.maxSpeedMP;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements.Line;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements.holdPos;
import org.firstinspires.ftc.teamcode.Subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.UniversalTools.RobotHardware;

@TeleOp
public class SemiautoTeleop extends ScheduledOpMode {
    RobotHardware robot;
    IceWaddler waddler;

    IntakeSubsystem intake;
    GamepadSubsystem gamepad;

    @Override
    public void init() {
        robot=new RobotHardware(hardwareMap);
        robot.init();

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(Position.ORIGIN,false);

        gamepad=new GamepadSubsystem(gamepad1, gamepad2);

        rootAction=new ActionParallel(NONE,
                gamepad.new Update(),
                new ActionSwitch(gamepad.driveSelect,
                        waddler.new VelDrive(gamepad.driveInput),
                        new ActionSeries(
                                waddler.new InitPath(),
                                waddler.new MotionAction(new Line(new PathingPoint(Position.ORIGIN),new maxSpeedMP(),new linearHP(),new String[]{})),
                                new ActionParallel(NONE,waddler.new MotionAction(new holdPos(new String[]{})))
                        )
                )
        );
    }
}
