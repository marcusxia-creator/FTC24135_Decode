package org.firstinspires.ftc.teamcode.Auto.Runs;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.*;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.IntakeRunMode;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.PoseStorage;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.Shooter;
import org.firstinspires.ftc.teamcode.TeleOps.Limelight;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.Turret;

import com.qualcomm.hardware.limelightvision.LLResult;

import java.util.List;

@Autonomous(name = "RedSideFarAuto", group = "Autonomous")
public class RedSideFarAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, 7.5, Math.toRadians(90));
    public RobotHardware robot;

    public int tagID = -1;
    public double bestArea = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        robot = new RobotHardware(hardwareMap);
        robot.init();
        Shooter shooter = new Shooter(robot);

        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();



        while (!isStopRequested() && !isStarted()) {
            LLResult llResult =  robot.limelight.getLatestResult();

            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                tagID = fiducial.getFiducialId(); // The ID number of the fiducial
            }
        }

        robot.spindexerServo.setPosition(spindexerSlot3);

        Action IntakeSet1Drive1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y), Math.toRadians(90))
                .build();

        Action IntakeSet1Drive2 = drive.actionBuilder(new Pose2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y,Math.toRadians(90)))
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
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position2_X, IntakeSet1Position2_Y),Math.toRadians(90))
                .build();
        Action DriveToShoot2 = drive.actionBuilder(new Pose2d(IntakeSet1Position2_X, IntakeSet1Position2_Y,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading))
                .build();

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(
                    shooter.ShooterOn(FarShotPower),
                    shooter.ShooterRun(FarShotPower, 2),
                    shooter.ShooterOff(),
                    new ParallelAction(
                        IntakeRun(),
                        new SequentialAction(
                            IntakeSet1Drive1,
                            IntakeSet1Drive2
                        )
                    ),
                    new ParallelAction(
                            DriveToShoot1,
                            shooter.ShooterOn(FarShotPower)
                    ),
                    shooter.ShooterRun(FarShotPower, 0.5),
                    shooter.ShooterOff(),
                        IntakeSet2Drive1,
                    new ParallelAction(
                        IntakeSet2Drive2,
                        IntakeRun()
                    ),
                    new ParallelAction(
                        DriveToShoot2,
                        shooter.ShooterOn(FarShotPower)
                    ),
                    shooter.ShooterRun(FarShotPower, 0.5),
                    shooter.ShooterOff(),
                    IntakeSet2Drive1,
                    new ParallelAction(
                            new SequentialAction(

                            ),
                            IntakeRun()
                    ),
                    new ParallelAction(
                            DriveToShoot2,
                            shooter.ShooterOn(FarShotPower)
                    )
                )
            );

            PoseStorage.currentPose = drive.localizer.getPose();
        }
    }

    public Action IntakeRun(){
        return new IntakeRunMode(robot);
    }

}
