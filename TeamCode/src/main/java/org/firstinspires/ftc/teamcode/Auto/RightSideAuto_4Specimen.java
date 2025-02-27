
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="RightSideAuto_4Specimen", group="org.firstinspires.ftc.teamcode.Auto")
@Config
public class RightSideAuto_4Specimen extends LinearOpMode {

    public static double highbar_x_coordinate = -3;
    public static double highbar_y_coordinate = -30.5;
    public static double highbar_x_coordinate2 = 0;
    public static double highbar_x_coordinate3 = 3;
    public static double specimen_pickup_x_coordinate = 29;
    public static double specimen_pickup_y_coordinate = -49;
    public static double first_sample_pickup_x_coordinate = 48;
    public static double first_sample_pickup_y_coordinate = -52;
    public static double second_sample_pickup_x_coordinate = 58;
    public static double second_sample_pickup_y_coordinate = -52;
    public static double clawOpenTimer = 0.2;
    public static double waitTimer = 0.25;
    public static double vSlideWaitTimer = 2*waitTimer;
    public static double hSlideWaitTimer =  0.35;
    public static double wristWaitTimer = 0.25;
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
                //extend slides to specimen scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })

                //drive to bar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {
                //    drive.setDrivePower(new Pose2d(0, 0, 0));
                //})
                //open claw
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.08, () -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Retract_Pos);
                })
                .waitSeconds(0.1)
                //back out of bar position -1st time
                //drop slides and put arm back to transfer position

        /** 2nd Segment --> grab 1st red samples for specimen*/
            /**Move to 1st sample*/
                .lineToLinearHeading(new Pose2d(first_sample_pickup_x_coordinate, first_sample_pickup_y_coordinate, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.8); //Move Slides Down
                })
                ///Extend slide to grab 1st sample
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                    Slides_Stop();
                })
                .waitSeconds(0.1) /// for testing purpose. should be 0.1
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)

                ///Transfer 1st sample
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () ->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })

                ///Drop sample into OB Zone
                .UNSTABLE_addTemporalMarkerOffset(0.5, () ->  {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
            /**Move to 2nd sample*/
                .lineToLinearHeading(new Pose2d(second_sample_pickup_x_coordinate, second_sample_pickup_y_coordinate, Math.toRadians(90)))
                ///Return to transfer position

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })

                ///Grab 2nd sample
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .waitSeconds(0.25+hSlideWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)                                                                       // 0.2s required to wait claw to close fully to secure grabbing of sample

                ///Transfer 2nd sample
                .addTemporalMarker(() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .waitSeconds(wristWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                ///Drop 2nd sample at OB Zone
                .waitSeconds(0.9)
                .addTemporalMarker(() -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(0.15)
                ///  ----> Return to transfer position
                .addTemporalMarker(() -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
        /** 3rd Segment --> move to Specimen pick up position*/
            /// ---->  Move to 2nd specimen on ground
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                /// ----> Grab 2nd specimen from ground
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                })
                .waitSeconds(0.5+hSlideWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2) //grab the specimen
                ///retract the intake arm and 0.2 sec later to retract slide
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                /// ---->  Transfer 2nd specimen while moving to highbar for 2nd specimen
                .UNSTABLE_addTemporalMarkerOffset(0.15,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
                })
                //  ----> extend vertical slides to scoring position - 2nd specimen
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(0.25)
                /// ---> move the high bar to 2nd specimen scoring
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate2, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                // ----> open claw and flat wrist - scored 2nd specimen
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Retract_Pos);
                })
                .waitSeconds(0.1)
            /// move to pick up for 3rd specimen
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                //deposit arm back to transfter position while robot moving
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                //drop slides and go back to transfer position
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.9);
                })
                .waitSeconds(vSlideWaitTimer-0.25)
                .addTemporalMarker(() -> {
                    Slides_Stop();
                })
            /// ----> 3rd specimen extend slide to pick position
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                })
                .waitSeconds(0.5+hSlideWaitTimer)
            /// ----> 3rd specimen pick
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)//grab the specimen
            /// ---->  3rd specimen Transfer
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
                })
                /// ----> 3rd specimen extend slides to scoring position
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()  -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(0.45)
                /// -----> 3rd specimen move to bar for hook while vertical slide moving up
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                // ----> open claw and flat wrist
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(clawOpenTimer-0.1)
                .addTemporalMarker(() -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Retract_Pos);
                })
            /// ----> move to specimen pick up position for 4th specimen
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                /// ---> extend slide and open claw to pick up
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                /// ---> extend slide to pick
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                })
                .waitSeconds(hSlideWaitTimer+0.2)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)
                /** 4.1 segment ----> Transfer 4th specimen*/
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
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(0.6)
                //extend slides to scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(vSlideWaitTimer)
                //move to highbar 3rd time
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //open claw and flat wrist
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(clawOpenTimer-0.1)
                .addTemporalMarker(() -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Retract_Pos);
                })
                .waitSeconds(0.1)
                /** ----> Extend slides to OB ZONE*/
                .addTemporalMarker(() -> {
                        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                        Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, 0.9);
                        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                })
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-90)))
                .waitSeconds(0.7)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);

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
        /**
         *         while (opModeIsActive() && !atposition) {
         *             telemetry.addData("Slide mode","Running");
         *             telemetry.update();
         *         }
         */

    }

    private void Slides_Stop(){
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
    }
}