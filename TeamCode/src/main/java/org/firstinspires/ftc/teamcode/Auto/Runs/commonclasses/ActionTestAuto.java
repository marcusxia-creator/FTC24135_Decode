package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.FarShotPower;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.kickerRetract;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.shooterAdjusterMax;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.spindexerSlot1;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
@Config
@Autonomous(name = "ActionTestAuto", group = "Autonomous")
public class ActionTestAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, -8, Math.toRadians(0));

    public RobotHardware robot;
    public AprilTagDetection aprilTagDetection;

    public AutoShooterFSM shooter;
    public AutoIntakeFSM intake;
    public AutoTurretDrive turret;
    public AutoLimelightDetection limelightDetection;
    public MecanumDrive drive;

    public int targetGreen;

    @Override
    public void runOpMode() throws InterruptedException {
        ///Init
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.turretInit();

        aprilTagDetection = new AprilTagDetection(robot);
        aprilTagDetection.limelightStart();

        turret = new AutoTurretDrive(robot);
        intake = new AutoIntakeFSM(robot);
        shooter = new AutoShooterFSM(robot);
        limelightDetection  = new AutoLimelightDetection(robot, aprilTagDetection);


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
        ///Start
        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(
                    turret.TurretRun(68),
                    intake.IntakeRun(8),
                    shooter.ShootFarZone(FarShotPower,2, 0,targetGreen),
                    shooter.ShooterOff(),
                    intake.IntakeRun(8),
                    shooter.ShootFarZone(FarShotPower,2, 1,targetGreen),
                    shooter.ShooterOff(),
                    intake.IntakeRun(8),
                    shooter.ShootFarZone(FarShotPower,2, 2,targetGreen),
                    shooter.ShooterOff()
                )
            );
        }
    }

}
