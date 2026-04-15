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
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoTurretDrive;

import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.PoseStorage;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.*;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

@Autonomous(name = "BlueSideCloseAuto", group = "Autonomous")
public class BlueSideCloseAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(-48, -58, Math.toRadians(45));

    public RobotHardware robot;

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

        turret = new AutoTurretDrive(robot);
        intake = new AutoIntakeFSM(robot);
        shooter = new AutoShooterFSM(robot);

        aprilTagDetection = new AprilTagDetection(robot);
        aprilTagDetection.limelightStart();

        if (opModeInInit()) {
            Actions.runBlocking(turret.TurretRun(65));
            robot.spindexerServo.setPosition(spindexerSlot1);
            robot.kickerServo.setPosition(kickerRetract);
            robot.shooterAdjusterServo.setPosition(shooterAdjusterMax);
            while (opModeInInit()&&!isStopRequested()) {
                aprilTagDetection.limelightDetect();
                targetGreen = aprilTagDetection.findGreenSlotCloseBlue();
                telemetry.addData("Detected ID",aprilTagDetection.tagID);
                telemetry.addData("Target Green Slot",targetGreen);
                telemetry.update();
            }
        }

        TrajectoryActionBuilder DriveToShoot1Builder = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y), Math.toRadians(CloseShootingPosition_Heading));

        TrajectoryActionBuilder IntakeSet1Drive1Builder = DriveToShoot1Builder.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(IntakeSet3Position1_X, IntakeSet3Position1_Y), Math.toRadians(-90));

        TrajectoryActionBuilder IntakeSet1Drive2Builder = IntakeSet1Drive1Builder.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(IntakeSet3Position2_X, IntakeSet3Position2_Y), Math.toRadians(-90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(IntakeSet3Position3_X, IntakeSet3Position3_Y), Math.toRadians(-90));

        TrajectoryActionBuilder DriveToShoot2Builder = IntakeSet1Drive2Builder.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y), Math.toRadians(CloseShootingPosition_Heading));

        TrajectoryActionBuilder IntakeSet2Drive1Builder = DriveToShoot2Builder.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(Close_IntakeSet2Position1_X, Close_IntakeSet2Position1_Y), Math.toRadians(-90));

        TrajectoryActionBuilder IntakeSet2Drive2Builder = IntakeSet2Drive1Builder.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(Close_IntakeSet2Position2_X, Close_IntakeSet2Position2_Y), Math.toRadians(-90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(Close_IntakeSet2Position3_X, Close_IntakeSet2Position3_Y), Math.toRadians(-90));

        TrajectoryActionBuilder DriveToShoot3Builder = IntakeSet2Drive2Builder.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(Close_IntakeSet2Position1_X, Close_IntakeSet2Position1_Y ), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y), Math.toRadians(CloseShootingPosition_Heading));

        TrajectoryActionBuilder LeaveDriveBuilder = DriveToShoot3Builder.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(CloseShootingPosition_X, CloseShootingPosition_Y-16), Math.toRadians(CloseShootingPosition_Heading));

        Action DriveToShoot1 = DriveToShoot1Builder.build();
        Action IntakeSet1Drive1 = IntakeSet1Drive1Builder.build();
        Action IntakeSet1Drive2 = IntakeSet1Drive2Builder.build();
        Action DriveToShoot2 = DriveToShoot2Builder.build();
        Action IntakeSet2Drive1 = IntakeSet2Drive1Builder.build();
        Action IntakeSet2Drive2 = IntakeSet2Drive2Builder.build();
        Action DriveToShoot3 = DriveToShoot3Builder.build();
        Action LeaveDrive = LeaveDriveBuilder.build();

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    turret.TurretRun(CloseTurretAngle),
                                    DriveToShoot1
                            ),
                            shooter.ShootCloseZone(CloseShotPower, 0.1,0,targetGreen),
                            shooter.ShooterOff(),
                            new ParallelAction(
                                    turret.TurretRun(CloseTurretAngle),
                                    intake.IntakeRun(6),
                                    new SequentialAction(
                                            IntakeSet1Drive1,
                                            IntakeSet1Drive2
                                    )
                            ),
                            new ParallelAction(
                                    DriveToShoot2,
                                    shooter.ShooterOn(CloseShotPower)
                            ),
                            shooter.ShootCloseZone(CloseShotPower, 0.1,2,targetGreen),
                            shooter.ShooterOff(),
                            new ParallelAction(
                                    turret.TurretRun(CloseTurretAngle),
                                    intake.IntakeRun(6),
                                    new SequentialAction(
                                            IntakeSet2Drive1,
                                            IntakeSet2Drive2
                                    )
                            ),
                            new ParallelAction(
                                    DriveToShoot3,
                                    shooter.ShooterOn(CloseShotPower)
                            ),
                            shooter.ShootCloseZone(CloseShotPower, 0.1,1,targetGreen),
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

