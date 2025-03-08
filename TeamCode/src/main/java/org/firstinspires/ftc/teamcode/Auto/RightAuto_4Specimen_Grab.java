
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="RightSideAuto_4Specimen_gw_test_Grab", group="org.firstinspires.ftc.teamcode.Auto")
@Config
public class RightAuto_4Specimen_Grab extends LinearOpMode {

    public static double highbar_x_coordinate = -3;
    public static double highbar_y_coordinate = -31.5;
    public static double highbar_x_coordinate2 = 0;
    public static double highbar_x_coordinate3 = 3;
    public static double specimen_pickup_x_coordinate = 29;
    public static double specimen_pickup_y_coordinate = -49;
    public static double first_sample_pickup_x_coordinate = 29.32;
    public static double first_sample_pickup_y_coordinate = -45.5;
    public static double second_sample_pickup_x_coordinate = 39.32;
    public static double second_sample_pickup_y_coordinate = -45.5;
    public static double clawOpenTimer = 0.2;
    public static double waitTimer = 0.25;
    public static double vSlideWaitTimer = 2*waitTimer;
    public static double hSlideWaitTimer =  0.35;
    public static double intakeArmWristWaitTimer = 0.25;
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable (hardwareMap);
        robot.init(hardwareMap);

        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_highbasketpause);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);

        Pose2d startPose = new Pose2d(7.5, -64, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                //drive to bar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate-6, highbar_y_coordinate, Math.toRadians(-90)))
                .build();

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                //extend slides to specimen scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                //drive to bar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate-3, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //open claw
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.08, () -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.1)
                /// retract deposit arm back to transfter while moving
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                /// retract deposit slide back to transfter while moving
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.8); //Move Slides Down
                })
            /** 2nd Segment --> GRAB SAMPLES FOR SPECIMENS*                                         ----Total:2secs/
                /**Move to 1st sample*/
                .lineToLinearHeading(new Pose2d(first_sample_pickup_x_coordinate, first_sample_pickup_y_coordinate, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                    Slides_Stop();
                })
                /// 1st sample -- Extending slides                                                  ----Total:4secs
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid + 0.15);
                })
                .waitSeconds(intakeArmWristWaitTimer + hSlideWaitTimer) // pause 0.6s to close claw
                /// 1st sample -- Grabbing
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .turn(Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                /**Move to 2nd sample                                                               ----total: 5.7secs */
                /// 2nd sample -- Moving
                .lineToLinearHeading(new Pose2d(second_sample_pickup_x_coordinate, second_sample_pickup_y_coordinate, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                /// 2nd sample -- Extending slides                                                  ----total: 8secs
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .waitSeconds(intakeArmWristWaitTimer + hSlideWaitTimer)                             /// 0.6s
                /// 2nd sample -- Grabbing
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)                   // 0.2s required to wait claw to close fully to secure grabbing of sample

                /// 2nd sample -- Transferring                                                      ----total: 8.8secs
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                // retract slide slightly overlap with intake arm retract to transfer
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                ///Drop 2nd sample at OB Zone
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5,() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                /// Return to transfer position - While Robot is moving - overlapping with next move
                .UNSTABLE_addTemporalMarkerOffset(1.6,() -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .waitSeconds(1.5) // 0.9 when open deposit claw and ensure sample dropped, then move robot, deposit arm back to transfer while robot moving
                /** 3rd SEGMENT --> MOVE TO SPECIMEN PICKUP POSITION                                ----total: 9.7sec */
                /// ---->  2nd SPECIMEN -- Move to 2nd specimen on ground - it takes 1seconds
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                /// ----> 2nd SPECIMEN -- extending slide to set point while robot is moving        ----total: 12.5 secs
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                /// ----> 2nd SPECIMEN -- extending slide fully after Robot fully stopped
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                })
                .waitSeconds(intakeArmWristWaitTimer+hSlideWaitTimer)                               /// wait 0.6s total - give 0.5 seconds to human player.
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2) //grab the specimen
                ///----> 2nd SPECIMEN -- Transferring                                               ---- total 13.5 secs
                //retract the intake arm
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                //0.15 sec later to retract slide
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.15, () -> {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
                })
                //extend vertical slides to scoring position - while robot is moving
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(1.15)                                                                  ///wait 0.55 sec until completion of transfer
                /// ---> 2nd SPECIMEN -- Moving to the high bar to 2nd specimen scoring             ----total 14 sec, moving need 2sec = 16s
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate2-6, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                /// ----> open claw and flat wrist - scored 2nd specimen                            ----total :16s
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.15) // wait 0.15s then move the robot.
                /// ----> 3rd SPECIMEN - moving to pick up for 3rd specimen                         ----total 17.5sec
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                //while robot moving, deposit arm back to transfter position
                .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                /// while robot moving, drop slides and go back to transfer position at the same time of deposit
                .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.9);
                })
                ///needs to review the timing                                                       ----total 19.5sec
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    Slides_Stop();
                })
                /// ----> 3rd SPECIMEN -- extending slide to pick position
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                })
                .waitSeconds(0.4+hSlideWaitTimer)
                /// ----> 3rd SPECIMEN -- Grabbing
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)//grab the specimen
                /// ---->  3rd SPECIMEN  -- Transferring                                            ----total 21sec
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.15, () -> {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
                })
                /// ---->  3rd SPECIMEN -- raise up slides to scoring position                      ----total 22.6sec
                .UNSTABLE_addTemporalMarkerOffset(1.25, ()  -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(1.1) // ---> the wait time is up to the point vertical slide rises up.
                /// ---->  3rd SPECIMEN -- move to bar for hook while vertical slide moving up
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3-6, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                /// ----> open claw and flat wrist                                                  ----total 24.6 sec
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () ->  {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.15) /// wait 0.15s then move the robot.                              ---- total 25sec
                /// ----> 4th SPECIMEN - Move to specimen pick up position                          (2 sec)
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.35,() -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                /// while robot moving, drop slides and go back to transfer position at the same time of deposit
                .UNSTABLE_addTemporalMarkerOffset(-0.35, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.9);
                })
                .waitSeconds(vSlideWaitTimer-0.35) ///needs to review the timing
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    Slides_Stop();
                })
                /// ----> 4th SPECIMEN -- extend slide to pick set position
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                /// ----> 4th SPECIMEN -- extend slide to pick position
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                })
                .waitSeconds(hSlideWaitTimer+0.4)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)
                /// ----> 4th SPECIMEN ---- Transferring/
                .addTemporalMarker(() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(1.1)
                //----> 4th SPECIMEN ---- rising slides to scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(0.25)
                //----> 4th SPECIMEN ---- moving to highbar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3-6, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //open claw and flat wrist
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.1)
                /** ----> Extend slides to OB ZONE*/
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()  -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.9);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                })
                .waitSeconds(0.7)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq1);

        while (opModeIsActive()) {
            telemetry.addData("Slides Position: ", robot.liftMotorLeft.getCurrentPosition());
        }
    }

    private void Slides_Move(int targetPosition, double speed) {
        int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
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