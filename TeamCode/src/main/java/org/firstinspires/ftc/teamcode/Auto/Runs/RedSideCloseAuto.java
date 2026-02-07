package org.firstinspires.ftc.teamcode.Auto.Runs;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoIntakeFSM;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoTurretDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.*;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;


@Autonomous(name = "RedSideCloseAuto", group = "Autonomous")
public class RedSideCloseAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(-68, 52, Math.toRadians(-45));

    public RobotHardware robot;

    public AutoIntakeFSM intake;
    public AutoShooterFSM shooter;
    public AutoTurretDrive turret;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new RobotHardware(hardwareMap);
        robot.init();

        turret = new AutoTurretDrive(robot);
        intake = new AutoIntakeFSM(robot);
        shooter = new AutoShooterFSM(robot);

        robot.spindexerServo.setPosition(spindexerSlot1);
        robot.kickerServo.setPosition(kickerRetract);

        Action DriveToShoot1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y),Math.toRadians(CloseShootingPosition_Heading))
                .build();

        Action IntakeSet1Drive1 = drive.actionBuilder(new Pose2d(CloseShootingPosition_X, CloseShootingPosition_Y, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(IntakeSet3Position1_X, IntakeSet3Position1_Y), Math.toRadians(90))
                .build();

        Action IntakeSet1Drive2 = drive.actionBuilder(new Pose2d(IntakeSet3Position1_X, IntakeSet3Position1_Y,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(IntakeSet3Position2_X,IntakeSet3Position2_Y),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(IntakeSet3Position3_X,IntakeSet3Position3_Y),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(IntakeSet3Position4_X,IntakeSet3Position4_Y),Math.toRadians(90))
                .build();

        Action DriveToShoot2 = drive.actionBuilder(new Pose2d(IntakeSet3Position4_X, IntakeSet3Position4_Y,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y),Math.toRadians(CloseShootingPosition_Heading))
                .build();

        Action IntakeSet2Drive1 = drive.actionBuilder(new Pose2d(CloseShootingPosition_X,CloseShootingPosition_Y,Math.toRadians(CloseShootingPosition_Heading)))
                .strafeToLinearHeading(new Vector2d(Close_IntakeSet2Position1_X, Close_IntakeSet2Position1_Y), Math.toRadians(90))
                .build();

        Action IntakeSet2Drive2 = drive.actionBuilder(new Pose2d(Close_IntakeSet2Position1_X, Close_IntakeSet2Position1_Y,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(Close_IntakeSet2Position2_X,Close_IntakeSet2Position2_Y),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(Close_IntakeSet2Position3_X,Close_IntakeSet2Position3_Y),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(Close_IntakeSet2Position4_X,Close_IntakeSet2Position4_Y),Math.toRadians(90))
                .build();

        Action DriveToShoot3 = drive.actionBuilder(new Pose2d(Close_IntakeSet2Position4_X, Close_IntakeSet2Position4_Y,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(Close_IntakeSet2Position4_X, Close_IntakeSet2Position4_Y-10),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y),Math.toRadians(CloseShootingPosition_Heading))
                .build();


        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                        new ParallelAction(
                            turret.TurretRun(45),
                            shooter.ShooterOn(CloseShotPower),
                            DriveToShoot1
                        ),
                        shooter.ShooterRun(CloseShotPower, 0.1,0),
                        shooter.ShooterOff(),
                        new ParallelAction(
                                intake.IntakeRun(2),
                                new SequentialAction(
                                        IntakeSet1Drive1,
                                        IntakeSet1Drive2
                                )
                        ),
                        new ParallelAction(
                                DriveToShoot2,
                                shooter.ShooterOn(CloseShotPower)
                        ),
                        shooter.ShooterRun(CloseShotPower, 0.1,0),
                        shooter.ShooterOff(),
                        new ParallelAction(
                                intake.IntakeRun(2),
                                new SequentialAction(
                                        IntakeSet2Drive1,
                                        IntakeSet2Drive2
                                )
                        ),
                        new ParallelAction(
                                DriveToShoot3,
                                shooter.ShooterOn(CloseShotPower)
                        ),
                        shooter.ShooterRun(CloseShotPower, 0.1,0),
                        shooter.ShooterOff()
                    )
            );
        }
    }
}

