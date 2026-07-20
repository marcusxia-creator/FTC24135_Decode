package org.firstinspires.ftc.teamcode.Auto.Runs;

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM.ShooterSortingRunMode.SORTINGSHOOTERSTATE.*;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.*;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoIntakeFSM;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoSpindexerContext;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoTurretDrive;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.PoseStorage;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name = "12BallSort_RedSideCloseAuto", group = "Autonomous")
public class RedSideClose12SortedAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(-38.5, 54, Math.toRadians(90));

    public RobotHardware robot;

    public AutoSpindexerContext context;
    public AutoIntakeFSM intake;
    public AutoShooterFSM shooter;
    public AutoTurretDrive turret;

    public AprilTagDetection aprilTagDetection;

    public int targetGreen;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.turretInit();

        context = new AutoSpindexerContext();

        turret = new AutoTurretDrive(robot);
        intake = new AutoIntakeFSM(robot,context);
        shooter = new AutoShooterFSM(robot,context);

        aprilTagDetection = new AprilTagDetection(robot);
        aprilTagDetection.limelightStart();

        if (opModeInInit()) {
            Actions.runBlocking(turret.TurretRun(160));
            robot.spindexerServo.setPosition(spindexerSlot1);
            robot.kickerServo.setPosition(kickerRetract);
            robot.shooterAdjusterServo.setPosition(shooterAdjusterMax);
            while (opModeInInit()&&!isStopRequested()) {
                aprilTagDetection.limelightDetect();
                context.targetGreenSlot = aprilTagDetection.findGreenSlot();
                telemetry.addData("Detected ID",aprilTagDetection.tagID);
                telemetry.addData("Target Green Slot",context.targetGreenSlot);
                telemetry.update();
            }
        }

        TrajectoryActionBuilder DriveToShoot1Builder = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-38.5,44),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(CloseShootingPosition_X,CloseShootingPosition_Y),Math.toRadians(-90));

        TrajectoryActionBuilder IntakeSet1Drive1Builder = DriveToShoot1Builder.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(Gate_IntakeSet2Position1_X,Gate_IntakeSet2Position1_Y),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(Gate_IntakeSet2Position4_X,Gate_IntakeSet2Position4_Y),Math.toRadians(90));


        TrajectoryActionBuilder DriveToShoot2Builder = IntakeSet1Drive1Builder.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y-3), Math.toRadians(CloseShootingPosition_Heading));

        TrajectoryActionBuilder IntakeSet2Drive1Builder = DriveToShoot2Builder.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(IntakeSet1Position1_X,IntakeSet1Position1_Y),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(IntakeSet1Position4_X,IntakeSet1Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot3Builder = IntakeSet2Drive1Builder.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y), Math.toRadians(CloseShootingPosition_Heading));

        TrajectoryActionBuilder IntakeSet3Drive1Builder = DriveToShoot3Builder.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(IntakeSet3Position1_X,IntakeSet3Position1_Y),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(IntakeSet3Position4_X,IntakeSet3Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot4Builder = IntakeSet3Drive1Builder.endTrajectory().fresh()
                .strafeToLinearHeading(
                        new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y),
                        Math.toRadians(CloseShootingPosition_Heading)
                );

        TrajectoryActionBuilder LeaveDriveBuilder = DriveToShoot4Builder.endTrajectory().fresh()
                .strafeToLinearHeading(
                        new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y + 16),
                        Math.toRadians(CloseShootingPosition_Heading)
                );

        Action DriveToShoot1 = DriveToShoot1Builder.build();
        Action IntakeSet1Drive1 = IntakeSet1Drive1Builder.build();
        Action DriveToShoot2 = DriveToShoot2Builder.build();
        Action IntakeSet2Drive1 = IntakeSet2Drive1Builder.build();
        Action DriveToShoot3 = DriveToShoot3Builder.build();
        Action IntakeSet3Drive1 = IntakeSet3Drive1Builder.build();
        Action DriveToShoot4 = DriveToShoot4Builder.build();
        Action LeaveDrive = LeaveDriveBuilder.build();

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    turret.TurretRun(45),
                                    shooter.ShooterOn(CloseShotPower),
                                    shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_INIT,SORTINGSHOOTER_RUN),
                                    DriveToShoot1
                            ),
                            shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_SWITCH,SORTINGSHOOTER_END),
                            shooter.ShooterOff(),
                            new ParallelAction(
                                    intake.IntakeRun(2.5),
                                    IntakeSet1Drive1
                            ),
                            new ParallelAction(
                                    DriveToShoot2,
                                    shooter.ShooterOn(CloseShotPower),
                                    shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_INIT,SORTINGSHOOTER_RUN)
                            ),
                            shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_SWITCH,SORTINGSHOOTER_END),
                            shooter.ShooterOff(),
                            new ParallelAction(
                                    intake.IntakeRun(5),
                                    IntakeSet2Drive1
                            ),
                            new ParallelAction(
                                    DriveToShoot3,
                                    shooter.ShooterOn(CloseShotPower),
                                    shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_INIT,SORTINGSHOOTER_RUN)
                            ),
                            shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_SWITCH,SORTINGSHOOTER_END),
                            shooter.ShooterOff(),
                            new ParallelAction(
                                    intake.IntakeRun(2.5),
                                    IntakeSet3Drive1
                            ),
                            new ParallelAction(
                                    DriveToShoot4,
                                    shooter.ShooterOn(CloseShotPower),
                                    shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_INIT,SORTINGSHOOTER_RUN)
                            ),
                            shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_SWITCH,SORTINGSHOOTER_END),
                            new ParallelAction(
                                    shooter.ShooterOff(),
                                    LeaveDrive
                            )
                    )
            );
            robot.pinpoint.update();
            drive.localizer.update();
            PoseStorage.currentPose = drive.localizer.getPose();
            PoseStorage.motifGreenPos = targetGreen;
        }
    }
}

