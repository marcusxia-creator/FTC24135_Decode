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
            robot.shooterAdjusterServo.setPosition(shooterAdjusterMax);
            while (opModeInInit()&&!isStopRequested()) {
                aprilTagDetection.limelightDetect();
                targetGreen = aprilTagDetection.findGreenSlotStandard();
                telemetry.addData("Detected ID",aprilTagDetection.tagID);
                telemetry.addData("Target Green Slot",targetGreen);
                telemetry.update();
            }
        }

        TrajectoryActionBuilder IntakeSet1Drive1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position1_X, IntakeSet1Position1_Y), Math.toRadians(90));

        TrajectoryActionBuilder IntakeSet1Drive2 = IntakeSet1Drive1.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(IntakeSet1Position2_X,IntakeSet1Position2_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(IntakeSet1Position3_X,IntakeSet1Position3_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot1 = IntakeSet1Drive2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        TrajectoryActionBuilder IntakeSet2Drive1 = DriveToShoot1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y), Math.toRadians(90));

        TrajectoryActionBuilder IntakeSet2Drive2 = IntakeSet2Drive1.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(Far_IntakeSet2Position2_X,Far_IntakeSet2Position2_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(Far_IntakeSet2Position3_X,Far_IntakeSet2Position3_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot2 = IntakeSet2Drive2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        Action intakeSet1Drive1Action = IntakeSet1Drive1.build();
        Action intakeSet1Drive2Action = IntakeSet1Drive2.build();
        Action driveToShoot1Action    = DriveToShoot1.build();
        Action intakeSet2Drive1Action = IntakeSet2Drive1.build();
        Action intakeSet2Drive2Action = IntakeSet2Drive2.build();
        Action driveToShoot2Action    = DriveToShoot2.build();

        waitForStart();

        if (!isStopRequested()) {
            Actions.runBlocking(
                new SequentialAction(
                    turret.TurretRun(FarTurretAngle1),
                    shooter.ShooterOn(FarShotPower1),
                    shooter.ShootFarZone(FarShotPower1, 1.2, 0,targetGreen),
                    shooter.ShooterOff(),
                    new ParallelAction(
                        intake.IntakeRun(12),
                        new SequentialAction(
                            intakeSet1Drive1Action,
                            intakeSet1Drive2Action
                        )
                    ),
                    new ParallelAction(
                            driveToShoot1Action,
                            shooter.ShooterOn(FarShotPower2),
                            turret.TurretRun(FarTurretAngle2)
                    ),
                    shooter.ShootFarZone(FarShotPower2, 0,0,targetGreen),
                    shooter.ShooterOff(),
                    new ParallelAction(
                        intake.IntakeRun(8),
                        new SequentialAction(
                            intakeSet2Drive1Action,
                            intakeSet2Drive2Action
                        )
                    ),
                    new ParallelAction(
                            driveToShoot2Action,
                            turret.TurretRun(FarTurretAngle2),
                            shooter.ShooterOn(FarShotPower2)
                    ),
                    shooter.ShootFarZone(FarShotPower2, 0, 1,targetGreen),
                    shooter.ShooterOff()
                )
            );
            robot.pinpoint.update();
            drive.localizer.update();

            PoseStorage.currentPose = drive.localizer.getPose();
            PoseStorage.turretEndTick = robot.turretMotor.getCurrentPosition();
            PoseStorage.motifGreenPos = targetGreen;
        }
    }

}
