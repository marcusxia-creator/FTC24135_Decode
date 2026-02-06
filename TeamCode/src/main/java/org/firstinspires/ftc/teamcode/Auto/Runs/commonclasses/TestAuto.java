package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.FarShotPower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, -8, Math.toRadians(0));

    public RobotHardware robot;
    public Shooter shooter;
    public Intake intake;
    public MecanumDrive drive;
    public AprilTagDetection aprilTagDetection;

    public int targetGreenSlot;
    public int initShootingSlot;

    @Override
    public void runOpMode() throws InterruptedException {
        ///Init
        aprilTagDetection = new AprilTagDetection(robot);
        drive = new MecanumDrive(hardwareMap, initialPose);
        robot = new RobotHardware(hardwareMap);
        shooter = new Shooter(robot);
        intake = new Intake(robot, targetGreenSlot);

        robot.init();
        aprilTagDetection.limelightStart();

        while (!isStopRequested() && !isStarted()) {
            aprilTagDetection.limelightDetect();
            targetGreenSlot = aprilTagDetection.findGreenSlot();
            telemetry.addData("AprilTag Detection", aprilTagDetection.tagID);
            telemetry.update();
        }

        ///Start
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(
                    intake.IntakeRun(targetGreenSlot),
                    TurretRun(90),
                    shooter.ShooterRun(FarShotPower,4,intake.getInitShotSlot()),
                    shooter.ShooterOff()
                )
            );
        }
    }

    public Action TurretRun (int targetAngle){
        return new TurretRunMode(robot, targetAngle);
    }

}
