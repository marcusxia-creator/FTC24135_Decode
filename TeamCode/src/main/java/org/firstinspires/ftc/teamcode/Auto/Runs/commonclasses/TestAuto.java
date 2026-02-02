package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.CloseShotPower;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.FarShotPower;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.Far_IntakeSet2Position1_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.Far_IntakeSet2Position1_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.Far_IntakeSet2Position2_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.Far_IntakeSet2Position2_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.Far_IntakeSet2Position3_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.Far_IntakeSet2Position3_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.Far_IntakeSet2Position4_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.Far_IntakeSet2Position4_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.IntakeSet1Position1_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.IntakeSet1Position1_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.IntakeSet1Position2_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.IntakeSet1Position2_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.IntakeSet1Position3_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.IntakeSet1Position3_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.IntakeSet1Position4_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.IntakeSet1Position4_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.PreloadShootingPosition_Heading;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.PreloadShootingPosition_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.PreloadShootingPosition_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.ShootingPosition_Heading;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.ShootingPosition_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.ShootingPosition_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.TransitionLocation_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.TransitionLocation_Y;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, -8, Math.toRadians(0));
    public RobotHardware robot;



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        robot = new RobotHardware(hardwareMap);
        robot.init();
        Shooter shooter = new Shooter(robot);
        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(
                    IntakeRun(),
                    //shooter.ShooterOn(FarShotPower),
                    shooter.ShooterRun(FarShotPower,4),
                    shooter.ShooterOff()
                )
            );
        }
    }

    public Action IntakeRun(){
        return new IntakeRunMode(robot);
    }

}
