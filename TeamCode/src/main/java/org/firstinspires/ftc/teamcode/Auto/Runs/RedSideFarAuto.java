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
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.IntakeRunMode;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.Shooter;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name = "RedSideFarAuto", group = "Autonomous")
public class RedSideFarAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, 8, Math.toRadians(0));
    public RobotHardware robot;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        robot = new RobotHardware(hardwareMap);
        robot.init();
        Shooter shooter = new Shooter(robot);


        robot.rightGateServo.setPosition(gateDown);
        robot.leftGateServo.setPosition(gateDown);
        robot.pushRampServo.setPosition(rampDownPos);
        robot.spindexerServo.setPosition(spindexerSlot2);

        Action DriveToShoot1 = drive.actionBuilder(initialPose)
            .strafeToLinearHeading(new Vector2d(PreloadShootingPosition_X,PreloadShootingPosition_Y), Math.toRadians(PreloadShootingPosition_Heading))
            .build();

        Action IntakeSet1Drive1 = drive.actionBuilder(initialPose)
            .strafeToLinearHeading(new Vector2d(IntakeSet1Position1_X, IntakeSet1Position1_Y), Math.toRadians(90))
            .build();

        Action IntakeSet1Drive2_1 = drive.actionBuilder(new Pose2d(IntakeSet1Position1_X, IntakeSet1Position1_Y,Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(IntakeSet1Position2_X,IntakeSet1Position2_Y),Math.toRadians(90))
            .build();

        Action IntakeSet1Drive2_2 = drive.actionBuilder(new Pose2d(IntakeSet1Position2_X,IntakeSet1Position2_Y,Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(IntakeSet1Position3_X,IntakeSet1Position3_Y),Math.toRadians(90))
            .build();

        Action IntakeSet1Drive2_3 = drive.actionBuilder(new Pose2d(IntakeSet1Position3_X,IntakeSet1Position3_Y,Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(IntakeSet1Position4_X,IntakeSet1Position4_Y),Math.toRadians(90))
            .build();

        Action DriveToShoot2 = drive.actionBuilder(new Pose2d(IntakeSet1Position4_X,IntakeSet1Position4_Y,Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(TransitionLocation_X,TransitionLocation_Y),Math.toRadians(ShootingPosition_Heading))
            .strafeToLinearHeading(new Vector2d(ShootingPosition_X,ShootingPosition_Y),Math.toRadians(ShootingPosition_Heading))
            .build();

        Action IntakeSet2Drive1 = drive.actionBuilder(new Pose2d(ShootingPosition_X,ShootingPosition_Y,Math.toRadians(ShootingPosition_Heading)))
            .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y), Math.toRadians(90))
            .build();

        Action IntakeSet2Drive2_1 = drive.actionBuilder(new Pose2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y,Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position2_X, Far_IntakeSet2Position2_Y),Math.toRadians(90))
            .build();

        Action IntakeSet2Drive2_2 = drive.actionBuilder(new Pose2d(Far_IntakeSet2Position2_X, Far_IntakeSet2Position2_Y,Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position3_X, Far_IntakeSet2Position3_Y),Math.toRadians(90))
            .build();

        Action IntakeSet2Drive2_3 = drive.actionBuilder(new Pose2d(Far_IntakeSet2Position3_X, Far_IntakeSet2Position3_Y,Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position4_X, Far_IntakeSet2Position4_Y),Math.toRadians(90))
            .build();

        Action DriveToShoot3 = drive.actionBuilder(new Pose2d(Far_IntakeSet2Position4_X, Far_IntakeSet2Position4_Y,Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(ShootingPosition_X,ShootingPosition_Y),Math.toRadians(ShootingPosition_Heading))
            .build();


        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                        shooter.ShooterOn(FarShotPower),
                        DriveToShoot1
                    ),
                    shooter.ShooterRun(FarShotPower, 2),
                    shooter.ShooterOff(),
                    IntakeSet1Drive1,
                    new ParallelAction(
                        new SequentialAction(
                            IntakeSet1Drive2_1,
                            IntakeSet1Drive2_2,
                            IntakeSet1Drive2_3
                        ),
                        IntakeRun()
                    ),
                    new ParallelAction(
                        shooter.ShooterOn(CloseShotPower),
                        DriveToShoot2
                    ),
                    shooter.ShooterRun(CloseShotPower,0.5),
                    shooter.ShooterOff(),
                    IntakeSet2Drive1,
                    new ParallelAction(
                            new SequentialAction(
                                    IntakeSet2Drive2_1,
                                    IntakeSet2Drive2_2,
                                    IntakeSet2Drive2_3
                            ),
                            IntakeRun()
                    ),
                    new ParallelAction(
                            shooter.ShooterOn(CloseShotPower),
                            DriveToShoot3
                    ),
                    shooter.ShooterRun(CloseShotPower,0.5),
                    shooter.ShooterOff()
                )
            );
        }
    }

    public Action IntakeRun(){
        return new IntakeRunMode(robot);
    }

}
