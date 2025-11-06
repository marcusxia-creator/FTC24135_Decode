package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOps.BallColor;
import org.firstinspires.ftc.teamcode.TeleOps.SlotList;
import org.firstinspires.ftc.teamcode.TeleOps.BallSlot;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.SharedColorSequence;
import org.firstinspires.ftc.teamcode.Vision.AprilTagUpdate;

import java.util.Arrays;
import java.util.List;

public class Auto_RR1_IntakeAndShoot3Balls extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Initial Pose2d
        Pose2d initialPose = new Pose2d(64, 8, Math.toRadians(-30));
        // === Hardware & Drive ===
        RobotHardware robot = new RobotHardware(hardwareMap);
        // set Road Runner params for slow movement
        MecanumDrive.PARAMS.maxWheelVel = 40;                    // inches/sec
        MecanumDrive.PARAMS.maxProfileAccel = 30;                 // inches/sec^2
        MecanumDrive.PARAMS.maxAngVel = Math.toRadians(120);      // rad/sec
        MecanumDrive.PARAMS.maxAngAccel = Math.toRadians(90);
        // set MecanumDrive to drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        
        // Initialize AprilTag detection
        AprilTagUpdate aprilTagUpdate = new AprilTagUpdate(hardwareMap);

        // === Spindexer Slots ===
        double[] slotAngles = {0.15, 0.45, 0.75};
        SlotList sharedBalls = new SlotList(slotAngles);
        List<BallSlot> ballSlots = sharedBalls.getBalls();

        // === Define key poses ===
        Pose2d startPose = initialPose;
        Pose2d pickupPose = new Pose2d(32, 32, Math.toRadians(90));
        Pose2d shootPose = new Pose2d(0, 0, Math.toRadians(45));
        BallColor[] targetSequence = {};

        AutoIntake autoIntake = new AutoIntake(robot, ballSlots,slotAngles);

        // === Build trajectories ===
        TrajectoryActionBuilder toPickup = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(32, 32))
                .turnTo(Math.toRadians(90))
                .endTrajectory();

        // small low-speed 2-inch movements forward (in inches)
        TrajectoryActionBuilder intakeSegment1 = toPickup.fresh()
                .lineToY(34);

        TrajectoryActionBuilder intakeSegment2 = intakeSegment1.endTrajectory().fresh()
                .lineToY(36);

        TrajectoryActionBuilder toShoot = intakeSegment2.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 0))
                .turnTo(Math.toRadians(45));

        Action moveToPickupAction = toPickup.build();
        Action moveSegment1 = intakeSegment1.build();
        Action moveSegment2 = intakeSegment2.build();
        Action moveToShootAction = toShoot.build();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        aprilTagUpdate.update();;
        SharedColorSequence.aprilTagSequence = aprilTagUpdate.getSequence();
        SharedColorSequence.detectedTagId = aprilTagUpdate.getTagID();
        if (isStopRequested()) return;

        // === Step 1: Move to intake area ===
        Actions.runBlocking(moveToPickupAction);


        // === Step 2: Begin Intake Sequence ===
        telemetry.addLine("Starting intake sequence...");
        telemetry.update();

        autoIntake.setState(AutoIntake.INTAKEBALLSTATE.INTAKE_SWEEPING);
        int lastBallCount = 0;

        // Run first 2-inch forward motion while intaking
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                moveSegment1,
                                moveSegment2),
                        autoIntake.collectBallsAction(3)
                )
        );

        // Move to the Shoot Position
        telemetry.addLine("Intake complete. Moving to shoot position...");      
        telemetry.update();
        Actions.runBlocking(moveToShootAction);
        
        telemetry.addData("Stored Sequence", Arrays.toString(SharedColorSequence.aprilTagSequence));
        telemetry.addLine("Auto Complete!");
        telemetry.update();
    }
}