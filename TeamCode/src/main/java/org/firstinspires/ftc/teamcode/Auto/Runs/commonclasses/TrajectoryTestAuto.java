package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.FarShootingPosition_Heading;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.FarShootingPosition_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.FarShootingPosition_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.Far_IntakeSet2Position1_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.Far_IntakeSet2Position1_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.Far_IntakeSet2Position2_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.Far_IntakeSet2Position2_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.Far_IntakeSet2Position3_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.Far_IntakeSet2Position3_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.Far_IntakeSet2Position4_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.Far_IntakeSet2Position4_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.IntakeSet1Position1_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.IntakeSet1Position1_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.IntakeSet1Position2_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.IntakeSet1Position2_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.IntakeSet1Position3_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.IntakeSet1Position3_Y;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.IntakeSet1Position4_X;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.IntakeSet1Position4_Y;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name = "TrajectoryTestAuto", group = "Autonomous")
public class TrajectoryTestAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, 7.5, Math.toRadians(90));

    public RobotHardware robot;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.turretInit();
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,64,7.5,AngleUnit.DEGREES,90));

        TrajectoryActionBuilder IntakeSet1Drive1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(IntakeSet1Position1_X, IntakeSet1Position1_Y), Math.toRadians(90));

        TrajectoryActionBuilder IntakeSet1Drive2 = IntakeSet1Drive1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position2_X,IntakeSet1Position2_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position3_X,IntakeSet1Position3_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position4_X,IntakeSet1Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot1 = IntakeSet1Drive2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        TrajectoryActionBuilder IntakeSet2Drive1 = DriveToShoot1.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y), Math.toRadians(90));

        TrajectoryActionBuilder IntakeSet2Drive2 = IntakeSet2Drive1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position2_X,Far_IntakeSet2Position2_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position3_X,Far_IntakeSet2Position3_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position4_X,Far_IntakeSet2Position4_Y),Math.toRadians(90));

        TrajectoryActionBuilder DriveToShoot2 = IntakeSet2Drive2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

        /*
        TrajectoryActionBuilder FullRun = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position2_X,Far_IntakeSet2Position2_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position3_X,Far_IntakeSet2Position3_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position4_X,Far_IntakeSet2Position4_Y),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(Far_IntakeSet2Position1_X, Far_IntakeSet2Position1_Y-8), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading))
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position1_X, IntakeSet1Position1_Y), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position2_X,IntakeSet1Position2_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position3_X,IntakeSet1Position3_Y),Math.toRadians(90))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(IntakeSet1Position4_X,IntakeSet1Position4_Y),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(FarShootingPosition_X, FarShootingPosition_Y),Math.toRadians(FarShootingPosition_Heading));

         */

        Action intakeSet1Drive1Action = IntakeSet1Drive1.build();
        Action intakeSet1Drive2Action = IntakeSet1Drive2.build();
        Action driveToShoot1Action    = DriveToShoot1.build();
        Action intakeSet2Drive1Action = IntakeSet2Drive1.build();
        Action intakeSet2Drive2Action = IntakeSet2Drive2.build();
        Action driveToShoot2Action    = DriveToShoot2.build();
        //Action fullRunAction          = FullRun.build();

        waitForStart();

        if (!isStopRequested()) {
            Actions.runBlocking(
                new SequentialAction(
                    intakeSet1Drive1Action,
                    intakeSet1Drive2Action,
                    driveToShoot1Action,
                    intakeSet2Drive1Action,
                    intakeSet2Drive2Action,
                    driveToShoot2Action
                )
            );
            robot.pinpoint.update();
            drive.localizer.update();
            telemetry.addData("Pose2d",drive.localizer.getPose());
            PoseStorage.currentPose = drive.localizer.getPose();
            PoseStorage.turretEndTick = robot.turretMotor.getCurrentPosition();
        }
    }

}
