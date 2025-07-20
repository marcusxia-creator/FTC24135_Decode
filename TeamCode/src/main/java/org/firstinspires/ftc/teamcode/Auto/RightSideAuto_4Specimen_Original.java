package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="RightSideAuto_4Specimen_Floor_Pick", group="org.firstinspires.ftc.teamcode.Auto")
@Config
public class RightSideAuto_4Specimen_Original extends LinearOpMode {

    public static double highbar_x_coordinate = -5;
    public static double highbar_y_coordinate = -32;
    public static double highbar_x_coordinate2 = -6;
    public static double highbar_x_coordinate3 = -6;
    public static double specimen_pickup_x_coordinate = 29.5;
    public static double specimen_pickup_y_coordinate = -46.5;
    public static double bar_out_point_1_X = 24;
    public static double bar_out_point_1_Y = -55;

    public static double first_sample_point_1_X = 30;
    public static double first_sample_point_1_Y = -44.5; //heading = 45

    public static double first_sample_point_2_X = 33;
    public static double first_sample_point_2_Y = -47; //heading = -45

    public static double second_sample_point_1_X = 41;
    public static double second_sample_point_1_Y = -45; //heading = 45

    public static double third_sample_point_1_X = 64.25;
    public static double third_sample_point_1_Y = -6;

    public static double third_sample_point_2_X = 64.25;
    public static double third_sample_point_2_Y = -58;

    public static double clawOpenTimer = 0.1;
    public static double waitTimer = 0.25;
    public static double vSlideWaitTimer = 0.5;
    public static double hSlideWaitTimer =  0.35;
    public static double wristWaitTimer = 0.25;
    RobotHardware robot = new RobotHardware(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot.init(hardwareMap);

        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Side_Drop);
        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Side_Drop);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Side_Drop);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        sleep(200);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
        robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook+0.1);
        robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook+0.1);
        sleep(1000);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);

        Pose2d startPose = new Pose2d(7.5, -64, Math.toRadians(-90));
        drive.pinpointSetPose(startPose);
        drive.setPoseEstimate(startPose);
        ///-----------------------------------------------------------------------

        ///-----------------------------------------------------------------------
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //extend slides to specimen scoring position
                //drive to bar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
                //back out of bar position -1st time
                .lineToLinearHeading(new Pose2d(bar_out_point_1_X, bar_out_point_1_Y, Math.toRadians(-90)))
                //drop slides and put arm back to transfer position
                ///-----------------------------------------------------------------------
                /** --> push ground sample for specimen*/
                .lineToLinearHeading(new Pose2d(first_sample_point_1_X, first_sample_point_1_Y, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(first_sample_point_2_X, first_sample_point_2_Y, Math.toRadians(-45)))       // drop 2nd sample to ob zone
                .lineToLinearHeading(new Pose2d(second_sample_point_1_X,second_sample_point_1_Y, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(second_sample_point_1_X,second_sample_point_1_Y, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(third_sample_point_1_X,third_sample_point_1_Y,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(third_sample_point_2_X,third_sample_point_2_Y,Math.toRadians(-90)))
                ///-----------------------------------------------------------------------
                /** --> start to pick specimen -------------------------- */
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate2, highbar_y_coordinate, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3, highbar_y_coordinate, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate-2, specimen_pickup_y_coordinate+2, Math.toRadians(-45)))
                /** ----> Retract Deposit slide and arm*/
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3, highbar_y_coordinate, Math.toRadians(-90)))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
        while (opModeIsActive()) {
            telemetry.addData("Slides Position", robot.liftMotorLeft.getCurrentPosition());
        }
    }

    private void Slides_Move(int targetPosition, double speed) {
        int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_SLIDES);
        robot.liftMotorLeft.setTargetPosition(targetTick);
        robot.liftMotorRight.setTargetPosition(targetTick);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
    }

    private void Slides_Stop(){
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
    }
}