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
    public static double highbar_x_coordinate2 = -2.5;
    public static double highbar_x_coordinate3 = -3.75;
    public static double specimen_pickup_x_coordinate = 29.5;
    public static double specimen_pickup_y_coordinate = -46.5;
    public static double bar_out_point_1_X = 24;
    public static double bar_out_point_1_Y = -55;

    public static double first_sample_point_1_X = 28;
    public static double first_sample_point_1_Y = -43.7; //heading = 45

    public static double first_sample_point_2_X = 33;
    public static double first_sample_point_2_Y = -47; //heading = -45

    public static double second_sample_point_1_X = 39;
    public static double second_sample_point_1_Y = -45; //heading = 45

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
        robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
        robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
        sleep(1000);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);

        Pose2d startPose = new Pose2d(7.5, -64, Math.toRadians(-90));
        drive.pinpointSetPose(startPose);
        drive.setPoseEstimate(startPose);
        ///-----------------------------------------------------------------------

        ///-----------------------------------------------------------------------
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //extend slides to specimen scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Rear_Highbar_Pos, 0.9);
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Rear_Hook);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Rear_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Rear_Hook);
                })
                //drive to bar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //open claw
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(clawOpenTimer)
                .addTemporalMarker(() -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.05)
                //back out of bar position -1st time
                .lineToLinearHeading(new Pose2d(bar_out_point_1_X, bar_out_point_1_Y, Math.toRadians(-90)))

                //drop slides and put arm back to transfer position
                .UNSTABLE_addTemporalMarkerOffset(-0.85, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.8); //Move Slides Down
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Slides_Stop();
                })
                ///-----------------------------------------------------------------------
                /** --> push ground sample for specimen*/
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .lineToLinearHeading(new Pose2d(first_sample_point_1_X, first_sample_point_1_Y, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid+0.15);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                })
                .waitSeconds(0.45)
                .addTemporalMarker(()->{
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                })
                .lineToLinearHeading(new Pose2d(first_sample_point_2_X, first_sample_point_2_Y, Math.toRadians(-45)))       // drop 2nd sample to ob zone
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));})
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(second_sample_point_1_X, second_sample_point_1_Y, Math.toRadians(40)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                })
                .waitSeconds(0.20)
                .addTemporalMarker(()->{
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.20)
                /** waiting for testing
                 .lineToLinearHeading(new Pose2d(first_sample_point_2_X, first_sample_point_2_Y, Math.toRadians(-45)))
                 */
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                })

                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(0.2)
                ///-----------------------------------------------------------------------
                /** --> start to pick specimen -------------------------- */
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    //robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    //robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.15,()->{
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                })
                .waitSeconds(1.25)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                /** --> Transfer 2nd specimen*/
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                })
                .waitSeconds (wristWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(1.0)
                //extend slides to scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Rear_Hook);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Rear_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Rear_Hook);
                })
                ///move to highbar score 2nd specimen
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate2, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //open claw and flat wrist
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(clawOpenTimer)
                .addTemporalMarker(() -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.1)
                /** --> go back to pickup position for 3rd specimen- */
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                //drop slides and go back to transfer position while robot is moving to pick up position
                .UNSTABLE_addTemporalMarkerOffset(-0.85, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.9);
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                    Slides_Stop();
                })
                /** --> pick up 3rd specimen*/
                //.lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    //robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    //robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.15,()->{
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                })
                .waitSeconds(1.25)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                /** --> Transfer 3rd specimen*/
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                })
                .waitSeconds(wristWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(1.0)
                //extend slides to scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Rear_Hook);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Rear_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Rear_Hook);
                })
                /** --> Score 3rd specimen*/
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //open claw and flat wrist
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(clawOpenTimer)
                .addTemporalMarker(() -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.05)
                /** --> Pick up 4th specimen*/
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate-2, specimen_pickup_y_coordinate+2, Math.toRadians(-45)))
                /** ----> Retract Deposit slide and arm*/
                .UNSTABLE_addTemporalMarkerOffset(-0.85, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.9);
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                /** --> pick up 4th specimen*/
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    //robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    //robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45,()->{
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                })
                .waitSeconds(0.65)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)
                /** --> Transfer 4th specimen*/
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                })
                .waitSeconds(wristWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(1.0)
                //extend slides to scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Rear_Hook);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Rear_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Rear_Hook);
                })
                /** --> Score 4th specimen*/
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //open claw and flat wrist
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(clawOpenTimer)
                .addTemporalMarker(() -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.5)
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