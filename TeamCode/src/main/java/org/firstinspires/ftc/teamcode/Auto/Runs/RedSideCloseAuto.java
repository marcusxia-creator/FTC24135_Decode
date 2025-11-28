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
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.IntakeRunMode;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.Shooter;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.*;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;


@Autonomous(name = "RedSideCloseAuto", group = "Autonomous")
public class RedSideCloseAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(-68, 52, Math.toRadians(-45));
    public RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        robot = new RobotHardware(hardwareMap);
        robot.init();
        Shooter shooter = new Shooter(robot);

        robot.spindexerServo.setPosition(spindexerSlot2);
        robot.rightGateServo.setPosition(gateDown);
        robot.leftGateServo.setPosition(gateDown);
        robot.pushRampServo.setPosition(rampDownPos);



        // Start one builder at the initial pose
        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose);

        Action DriveToShoot1 = tab
                .strafeToLinearHeading(
                        new Vector2d(ShootingPosition_X, ShootingPosition_Y),
                        Math.toRadians(ShootingPosition_Heading)
                )
                .build();

        tab = tab.fresh();

        Action IntakeSet1Drive1 = tab
                .strafeToLinearHeading(
                        new Vector2d(IntakeSet3Position1_X, IntakeSet3Position1_Y),
                        Math.toRadians(90)
                )
                .build();

        tab = tab.fresh();

        Action IntakeSet1Drive2_1 = tab
                .strafeToLinearHeading(
                        new Vector2d(IntakeSet3Position2_X, IntakeSet3Position2_Y),
                        Math.toRadians(90)
                )
                .build();

        tab = tab.fresh();

        Action IntakeSet1Drive2_2 = tab
                .strafeToLinearHeading(
                        new Vector2d(IntakeSet3Position3_X, IntakeSet3Position3_Y),
                        Math.toRadians(90)
                )
                .build();

        tab = tab.fresh();

        Action IntakeSet1Drive2_3 = tab
                .strafeToLinearHeading(
                        new Vector2d(IntakeSet3Position4_X, IntakeSet3Position4_Y),
                        Math.toRadians(90)
                )
                .build();

        tab = tab.fresh();

        Action DriveToShoot2 = tab
                .strafeToLinearHeading(
                        new Vector2d(ShootingPosition_X, ShootingPosition_Y),
                        Math.toRadians(ShootingPosition_Heading)
                )
                .build();

        tab = tab.fresh();

        Action IntakeSet2Drive1 = tab
                .strafeToLinearHeading(
                        new Vector2d(Close_IntakeSet2Position1_X, Close_IntakeSet2Position1_Y),
                        Math.toRadians(90)
                )
                .build();

        tab = tab.fresh();

        Action IntakeSet2Drive2_1 = tab
                .strafeToLinearHeading(
                        new Vector2d(Close_IntakeSet2Position2_X, Close_IntakeSet2Position2_Y),
                        Math.toRadians(90)
                )
                .build();

        tab = tab.fresh();

        Action IntakeSet2Drive2_2 = tab
                .strafeToLinearHeading(
                        new Vector2d(Close_IntakeSet2Position3_X, Close_IntakeSet2Position3_Y),
                        Math.toRadians(90)
                )
                .build();

        tab = tab.fresh();

        Action IntakeSet2Drive2_3 = tab
                .strafeToLinearHeading(
                        new Vector2d(Close_IntakeSet2Position4_X, Close_IntakeSet2Position4_Y),
                        Math.toRadians(90)
                )
                .build();

        tab = tab.fresh();

        Action DriveToShoot3 = tab
                .strafeToLinearHeading(
                        new Vector2d(ShootingPosition_X, ShootingPosition_Y),
                        Math.toRadians(ShootingPosition_Heading)
                )
                .build();



        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    DriveToShoot1,
                                    shooter.ShooterOn(CloseShotPower)
                            ),
                            shooter.ShooterRun(CloseShotPower, 0.4),
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
                                    DriveToShoot2,
                                    shooter.ShooterOn(CloseShotPower)
                            ),
                            shooter.ShooterRun(CloseShotPower, 0.4),
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
                                    DriveToShoot3,
                                    shooter.ShooterOn(CloseShotPower)
                            ),
                            shooter.ShooterRun(CloseShotPower, 0.4),
                            shooter.ShooterOff()
                    )
            );
        }
    }

    public Action IntakeRun() {
        return new IntakeRunMode(robot);
    }
}
