package org.firstinspires.ftc.teamcode.Auto.Runs;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.*;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoIntakeFSM;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoTurretDrive;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.PoseStorage;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name = "RedSideFarAuto", group = "Autonomous")
public class RedSideFarAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, 7.5, Math.toRadians(90));

    public int targetGreen;

    public RobotHardware robot;

    public AutoIntakeFSM intake;
    public AutoShooterFSM shooter;
    public AutoTurretDrive turret;
    public AprilTagDetection aprilTagDetection;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new RobotHardware(hardwareMap);
        robot.init();

        turret = new AutoTurretDrive(robot);
        intake = new AutoIntakeFSM(robot);
        shooter = new AutoShooterFSM(robot);

        aprilTagDetection = new AprilTagDetection(robot);
        aprilTagDetection.limelightStart();

        if (opModeInInit()) {
            Actions.runBlocking(turret.TurretRun(90));
            robot.spindexerServo.setPosition(spindexerSlot1);
            robot.kickerServo.setPosition(kickerRetract);
            aprilTagDetection.limelightDetect(opModeInInit()&&!isStopRequested());
            targetGreen = aprilTagDetection.tagID;
        }

        Action IntakeSet1Drive1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y), Math.toRadians(90))
                .build();

        Action IntakeSet1Drive2 = drive.actionBuilder(new Pose2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position2_X,Far_IntakeSet2Position2_Y),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position3_X,Far_IntakeSet2Position3_Y),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position4_X,Far_IntakeSet2Position4_Y),Math.toRadians(90))
                .build();

        Action DriveToShoot1 = drive.actionBuilder(new Pose2d(Far_IntakeSet2Position4_X,Far_IntakeSet2Position4_Y,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y-8), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading))
                .build();

        Action IntakeSet2Drive1 = drive.actionBuilder(new Pose2d(FarShootingPosition_X,FarShootingPosition_Y,Math.toRadians(FarShootingPosition_Heading)))
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position1_X, IntakeSet1Position1_Y), Math.toRadians(90))
                .build();

        Action IntakeSet2Drive2 = drive.actionBuilder(new Pose2d(IntakeSet1Position1_X, IntakeSet1Position1_Y,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position2_X,IntakeSet1Position2_Y),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position3_X,IntakeSet1Position3_Y),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position4_X,IntakeSet1Position4_Y),Math.toRadians(90))
                .build();
        Action DriveToShoot2 = drive.actionBuilder(new Pose2d(IntakeSet1Position4_X, IntakeSet1Position4_Y,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading))
                .build();

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(
                    turret.TurretRun(66),
                    shooter.ShooterOn(FarShotPower),
                    shooter.ShooterRun(FarShotPower, 1.5,0),
                    shooter.ShooterOff(),
                    new ParallelAction(
                        intake.IntakeRun(targetGreen),
                        new SequentialAction(
                            IntakeSet1Drive1,
                            IntakeSet1Drive2
                        )
                    ),
                    new ParallelAction(
                            DriveToShoot1,
                            shooter.ShooterOn(FarShotPower)
                    ),
                    shooter.ShooterRun(FarShotPower, 0.1,intake.getInitShotSlot()),
                    shooter.ShooterOff(),
                    new ParallelAction(
                            intake.IntakeRun(targetGreen),
                            new SequentialAction(
                                    IntakeSet2Drive1,
                                    IntakeSet2Drive2
                            )
                    ),
                    new ParallelAction(
                            DriveToShoot2,
                            shooter.ShooterOn(FarShotPower)
                    ),
                    shooter.ShooterRun(FarShotPower, 0.1, intake.getInitShotSlot()),
                    shooter.ShooterOff(),
                    turret.TurretRun(0)
                )
            );

            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }

}
