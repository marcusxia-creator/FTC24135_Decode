package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.BlueSidePositions.FarShotPower;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.kickerRetract;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.shooterAdjusterMax;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.spindexerSlot1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, -8, Math.toRadians(0));

    public RobotHardware robot;
    public AutoShooterFSM shooter;
    public AutoIntakeFSM intake;
    public AutoTurretDrive turret;
    public MecanumDrive drive;
    public AprilTagDetection aprilTagDetection;

    public int targetGreen;

    @Override
    public void runOpMode() throws InterruptedException {
        ///Init
        aprilTagDetection = new AprilTagDetection(robot);
        drive = new MecanumDrive(hardwareMap, initialPose);
        robot = new RobotHardware(hardwareMap);
        shooter = new AutoShooterFSM(robot);
        intake = new AutoIntakeFSM(robot);
        turret = new AutoTurretDrive(robot);

        robot.init();

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
                    turret.TurretRun(72),
                    intake.IntakeRun(targetGreen,8),
                    shooter.ShootFarZone(FarShotPower,1.5,intake.getInitShotSlot()),
                    shooter.ShooterOff()
                )
            );
        }
    }

}
