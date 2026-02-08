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

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
        robot.turretInit();
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,64,7.5,AngleUnit.DEGREES,90));

        turret = new AutoTurretDrive(robot);
        intake = new AutoIntakeFSM(robot);
        shooter = new AutoShooterFSM(robot);

        aprilTagDetection = new AprilTagDetection(robot);
        aprilTagDetection.limelightStart();

        if (opModeInInit()) {
            Actions.runBlocking(turret.TurretRun(90));
            robot.spindexerServo.setPosition(spindexerSlot1);
            robot.kickerServo.setPosition(kickerRetract);
            while (opModeInInit()&&!isStopRequested()) {
                aprilTagDetection.limelightDetect();
                targetGreen = aprilTagDetection.tagID;
                telemetry.addData("Detected ID",targetGreen);
                telemetry.update();
            }
        }

        TrajectoryActionBuilder IntakeSet1Drive1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y), Math.toRadians(90));

        TrajectoryActionBuilder IntakeSet1Drive2 = IntakeSet1Drive1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position2_X,Far_IntakeSet2Position2_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position3_X,Far_IntakeSet2Position3_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position4_X,Far_IntakeSet2Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot1 = IntakeSet1Drive2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y-8), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        TrajectoryActionBuilder IntakeSet2Drive1 = DriveToShoot1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position1_X, IntakeSet1Position1_Y), Math.toRadians(90));

        TrajectoryActionBuilder IntakeSet2Drive2 = IntakeSet2Drive1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position2_X,IntakeSet1Position2_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position3_X,IntakeSet1Position3_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position4_X,IntakeSet1Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot2 = IntakeSet2Drive2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        Action intakeSet1Drive1Action = IntakeSet1Drive1.build();
        Action intakeSet1Drive2Action = IntakeSet1Drive2.build();
        Action driveToShoot1Action    = DriveToShoot1.build();
        Action intakeSet2Drive1Action = IntakeSet2Drive1.build();
        Action intakeSet2Drive2Action = IntakeSet2Drive2.build();
        Action driveToShoot2Action    = DriveToShoot2.build();

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(
                    turret.TurretRun(68),
                    shooter.ShooterOn(FarShotPower),
                    shooter.ShooterRun(FarShotPower, 2,0),
                    shooter.ShooterOff(),
                    new ParallelAction(
                        intake.IntakeRun(targetGreen),
                        new SequentialAction(
                            intakeSet1Drive1Action,
                            intakeSet1Drive2Action
                        )
                    ),
                    new ParallelAction(
                            driveToShoot1Action,
                            shooter.ShooterOn(FarShotPower)
                    ),
                    shooter.ShooterRun(FarShotPower, 0.1,intake.getInitShotSlot()),
                    shooter.ShooterOff(),
                    new ParallelAction(
                        intake.IntakeRun(targetGreen),
                        new SequentialAction(
                            intakeSet2Drive1Action,
                            intakeSet2Drive2Action
                        )
                    ),
                    new ParallelAction(
                            driveToShoot2Action,
                            shooter.ShooterOn(FarShotPower)
                    ),
                    shooter.ShooterRun(FarShotPower, 0.1, intake.getInitShotSlot()),
                    shooter.ShooterOff(),
                    turret.TurretRun(0)
                )
            );
            robot.pinpoint.update();
            drive.localizer.update();
            PoseStorage.currentPose = drive.localizer.getPose();
            PoseStorage.motifGreenPos = targetGreen;
        }
    }

}
