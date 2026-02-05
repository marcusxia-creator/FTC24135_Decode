package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.FarShotPower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
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
                    TurretRun(90),
                    IntakeRun(),
                    shooter.ShooterRun(FarShotPower,4,),
                    shooter.ShooterOff()
                )
            );
        }
    }

    public Action TurretRun (int targetAngle){
        return new TurretRunMode(robot, targetAngle);
    }

}
