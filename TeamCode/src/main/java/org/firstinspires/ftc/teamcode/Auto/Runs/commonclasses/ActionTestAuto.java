package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM.ShooterRapidRunMode.SHOOTERSTATE.SHOOTER_END;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM.ShooterRapidRunMode.SHOOTERSTATE.SHOOTER_INIT;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM.ShooterRapidRunMode.SHOOTERSTATE.SHOOTER_RESET;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM.ShooterRapidRunMode.SHOOTERSTATE.SHOOTER_SWITCH;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM.ShooterRapidRunMode.SHOOTERSTATE.SHOOTER_WAIT;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.CloseShotPower;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.RedSidePositions.FarShotPower;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AutoColorDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import static org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.AutoShooterFSM.ShooterSortingRunMode.SORTINGSHOOTERSTATE.*;

@Config
@Autonomous(name = "ActionTestAuto", group = "Autonomous")
public class ActionTestAuto extends LinearOpMode {
    public static Pose2d initialPose = new Pose2d(64, -8, Math.toRadians(0));

    public RobotHardware robot;
    public AutoShooterFSM shooter;
    public AutoIntakeFSM intake;
    public AutoTurretDrive turret;
    public MecanumDrive drive;
    public AprilTagDetection aprilTagDetection;
    public AutoSpindexerContext context;
    public AutoColorDetection colorDetection;

    @Override
    public void runOpMode() throws InterruptedException {
        ///Init
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.turretInit();

        context = new AutoSpindexerContext();
        turret = new AutoTurretDrive(robot);
        intake = new AutoIntakeFSM(robot,context);
        shooter = new AutoShooterFSM(robot,context);

        colorDetection = new AutoColorDetection(robot);


        aprilTagDetection = new AprilTagDetection(robot);
        aprilTagDetection.limelightStart();

        if (opModeInInit()) {
            robot.spindexerServo.setPosition(spindexerSlot0);
            robot.kickerServo.setPosition(kickerRetract);
            robot.shooterAdjusterServo.setPosition(shooterAdjusterMax);
            colorDetection.detectInit();
            while (opModeInInit()&&!isStopRequested()) {
                //Actions.runBlocking(turret.TurretRun(68));
                colorDetection.updateSlotColors();
                aprilTagDetection.limelightDetect();
                context.currentGreenSlot = colorDetection.findGreenSlot();
                context.targetGreenSlot = aprilTagDetection.findGreenSlot();
                context.updateShootingInitSlot();
                telemetry.addData("Detected ID",aprilTagDetection.tagID);
                telemetry.addData("Target Green Slot",context.targetGreenSlot);
                telemetry.addData("Current Green Slot",colorDetection.findGreenSlot());
                telemetry.addData("Slot 1 Color",colorDetection.getSlotColor(0));
                telemetry.addData("Slot 2 Color",colorDetection.getSlotColor(1));
                telemetry.addData("Slot 3 Color",colorDetection.getSlotColor(2));
                telemetry.addData("Green Slot", context.currentGreenSlot);
                telemetry.addData("Calc Starting Slot", context.shootingInitSlot);
                telemetry.update();
            }
        }
        ///Start
        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                new SequentialAction(
                        shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_INIT,SORTINGSHOOTER_END),
                        intake.IntakeRun(8),
                        shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_INIT,SORTINGSHOOTER_END),
                        intake.IntakeRun(8),
                        shooter.ShootSorting(CloseShotPower, 0.1,SORTINGSHOOTER_INIT,SORTINGSHOOTER_END),
                        intake.IntakeRun(8)
                )
            );
        }
    }

}
