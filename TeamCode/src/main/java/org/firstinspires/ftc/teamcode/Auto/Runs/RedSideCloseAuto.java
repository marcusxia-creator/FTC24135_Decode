package org.firstinspires.ftc.teamcode.Auto.Runs;

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

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.*;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

@Autonomous(name = "RedSideCloseAuto", group = "Autonomous")
public class RedSideCloseAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(-40.5, 55, Math.toRadians(90));

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
            Actions.runBlocking(turret.TurretRun(45));
            robot.spindexerServo.setPosition(spindexerSlot1);
            robot.kickerServo.setPosition(kickerRetract);
            robot.shooterAdjusterServo.setPosition(shooterAdjusterMax);
            while (opModeInInit()&&!isStopRequested()) {
                aprilTagDetection.limelightDetect();
                targetGreen = aprilTagDetection.findGreenSlot();
                telemetry.addData("Detected ID",aprilTagDetection.tagID);
                telemetry.addData("Target Green Slot",targetGreen);
                telemetry.update();
            }
        }

        TrajectoryActionBuilder DriveToShoot1Builder = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-40.5,44),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(CloseShootingPosition_X,CloseShootingPosition_Y),Math.toRadians(-90));

// FIRST RUN SET: go to Close Set 2 instead of Intake Set 3
        TrajectoryActionBuilder IntakeSet1Drive1Builder = DriveToShoot1Builder.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(Close_IntakeSet2Position1_X,Close_IntakeSet2Position1_Y),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(Close_IntakeSet2Position4_X,Close_IntakeSet2Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot2Builder = IntakeSet1Drive1Builder.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y), Math.toRadians(CloseShootingPosition_Heading));

// SECOND RUN SET: go to the gate
        TrajectoryActionBuilder IntakeGateSet1Drive1Builder = DriveToShoot2Builder.endTrajectory().fresh()
                .setTangent(Math.toRadians(52))
                .splineTo(new Vector2d(GateIntakePosition_X,GateIntakePosition_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot3Builder = IntakeGateSet1Drive1Builder.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(CloseShootingPosition_X,CloseShootingPosition_Y + 16),Math.toRadians(-128));

// THIRD RUN SET: go to the furthest Close Set 2 position
        TrajectoryActionBuilder IntakeSet2Drive1Builder = DriveToShoot3Builder.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(IntakeSet3Position1_X,IntakeSet3Position1_Y),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(IntakeSet3Position4_X,IntakeSet3Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot4Builder = IntakeSet2Drive1Builder.endTrajectory().fresh()
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
        Action IntakeGateSet1Drive1 = IntakeGateSet1Drive1Builder.build();
        Action DriveToShoot3 = DriveToShoot3Builder.build();
        Action IntakeSet2Drive1 = IntakeSet2Drive1Builder.build();
        Action DriveToShoot4 = DriveToShoot4Builder.build();
        Action LeaveDrive = LeaveDriveBuilder.build();

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                        new ParallelAction(
                            turret.TurretRun(45),
                            shooter.ShooterOn(CloseShotPower),
                            DriveToShoot1
                        ),
                        shooter.ShootCloseZone(CloseShotPower, 0.1),
                        shooter.ShooterOff(),
                        new ParallelAction(
                                intake.IntakeRun(5),
                                new SequentialAction(
                                        IntakeSet1Drive1
                                )
                        ),
                        new ParallelAction(
                                DriveToShoot2,
                                shooter.ShooterOn(CloseShotPower)
                        ),
                        shooter.ShootCloseZone(CloseShotPower, 0.1),
                        shooter.ShooterOff(),
                        new ParallelAction(
                                intake.IntakeRun(8),
                                new SequentialAction(
                                    IntakeGateSet1Drive1
                                )
                        ),
                        new ParallelAction(
                                DriveToShoot3,
                                shooter.ShooterOn(CloseShotPower)
                        ),
                        shooter.ShootCloseZone(CloseShotPower, 0.1),
                        new ParallelAction(
                                intake.IntakeRun(5),
                                new SequentialAction(
                                        IntakeSet2Drive1
                                )
                        ), 
                        new ParallelAction(
                            DriveToShoot4,
                            shooter.ShooterOn(CloseShotPower)
                        ),
                        shooter.ShootCloseZone(CloseShotPower, 0.1),
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

