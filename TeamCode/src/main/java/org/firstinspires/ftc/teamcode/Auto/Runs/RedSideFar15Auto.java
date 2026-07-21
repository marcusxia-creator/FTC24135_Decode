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
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM.ShooterRapidRunMode.SHOOTERSTATE.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoIntakeFSM;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoSpindexerContext;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoTurretDrive;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.PoseStorage;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name = "15Ball_RedSideFarAuto", group = "Autonomous")
public class RedSideFar15Auto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, 7.5, Math.toRadians(90));

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
            Actions.runBlocking(turret.TurretRun(90));
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

        TrajectoryActionBuilder IntakeSet1Drive1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(IntakeSet1Position1_X, IntakeSet1Position1_Y), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(IntakeSet1Position4_X,IntakeSet1Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot1 = IntakeSet1Drive1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        TrajectoryActionBuilder IntakeHPSet1Drive1 = DriveToShoot1.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(IntakeHPPosition4_X,IntakeHPPosition4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot2 = IntakeHPSet1Drive1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        TrajectoryActionBuilder IntakeHPSet2Drive1 = DriveToShoot2.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(IntakeHP2Position4_X,IntakeHP2Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot3 = IntakeHPSet2Drive1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        TrajectoryActionBuilder IntakeHPSet3Drive1 = DriveToShoot3.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(IntakeHPPosition4_X,IntakeHPPosition4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot4 = IntakeHPSet3Drive1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        Action intakeSet1Drive1Action = IntakeSet1Drive1.build();
        Action driveToShoot1Action    = DriveToShoot1.build();
        Action intakeHPSet1Drive1Action = IntakeHPSet1Drive1.build();
        Action driveToShoot2Action    = DriveToShoot2.build();
        Action intakeHPSet2Drive1Action = IntakeHPSet2Drive1.build();
        Action driveToShoot3Action    = DriveToShoot3.build();
        Action intakeHPSet3Drive1Action = IntakeHPSet3Drive1.build();
        Action driveToShoot4Action    = DriveToShoot4.build();

        waitForStart();

        if (!isStopRequested()) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    turret.TurretRun(67),
                                    shooter.ShooterOn(FarShotPower),
                                    shooter.ShootFarZone(FarShotPower, 1.6,SHOOTER_INIT,SHOOTER_RUN)
                            ),
                            shooter.ShootFarZone(FarShotPower, 0,SHOOTER_SWITCH,SHOOTER_WAIT),
                            new ParallelAction(
                                    new SequentialAction(
                                            shooter.ShootFarZone(FarShotPower, 0,SHOOTER_RESET,SHOOTER_END),
                                            intake.IntakeRun(4)
                                    ),
                                    shooter.ShooterOff(),
                                    intakeSet1Drive1Action
                            ),
                            new ParallelAction(
                                    driveToShoot1Action,
                                    turret.TurretRun(67),
                                    shooter.ShooterOn(FarShotPower),
                                    shooter.ShootFarZone(FarShotPower, 1.6,SHOOTER_INIT,SHOOTER_RUN)
                            ),
                            shooter.ShootFarZone(FarShotPower, 0,SHOOTER_SWITCH,SHOOTER_WAIT),
                            new ParallelAction(
                                    new SequentialAction(
                                            shooter.ShootFarZone(FarShotPower, 0,SHOOTER_RESET,SHOOTER_END),
                                            intake.IntakeRun(4)
                                    ),
                                    intakeHPSet1Drive1Action
                            ),
                            new ParallelAction(
                                    driveToShoot2Action,
                                    turret.TurretRun(67),
                                    shooter.ShooterOn(FarShotPower),
                                    shooter.ShootFarZone(FarShotPower, 1.6,SHOOTER_INIT,SHOOTER_RUN)
                            ),
                            shooter.ShootFarZone(FarShotPower, 0,SHOOTER_SWITCH,SHOOTER_WAIT),
                            new ParallelAction(
                                    new SequentialAction(
                                            shooter.ShootFarZone(FarShotPower, 0,SHOOTER_RESET,SHOOTER_END),
                                            intake.IntakeRun(4)
                                    ),
                                    intakeHPSet2Drive1Action
                            ),
                            new ParallelAction(
                                    driveToShoot3Action,
                                    turret.TurretRun(67),
                                    shooter.ShooterOn(FarShotPower),
                                    shooter.ShootFarZone(FarShotPower, 1.6,SHOOTER_INIT,SHOOTER_RUN)
                            ),
                            shooter.ShootFarZone(FarShotPower, 0,SHOOTER_SWITCH,SHOOTER_WAIT),
                            new ParallelAction(
                                    new SequentialAction(
                                            shooter.ShootFarZone(FarShotPower, 0,SHOOTER_RESET,SHOOTER_END),
                                            intake.IntakeRun(4)
                                    ),
                                    intakeHPSet3Drive1Action
                            ),
                            new ParallelAction(
                                    driveToShoot4Action,
                                    turret.TurretRun(67),
                                    shooter.ShooterOn(FarShotPower),
                                    shooter.ShootFarZone(FarShotPower, 1.6,SHOOTER_INIT,SHOOTER_RUN)
                            ),
                            shooter.ShootFarZone(FarShotPower, 0.1,SHOOTER_SWITCH,SHOOTER_END),
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

