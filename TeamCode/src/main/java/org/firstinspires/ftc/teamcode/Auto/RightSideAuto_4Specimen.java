
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="RightSideAuto_4Specimen_gw_test", group="org.firstinspires.ftc.teamcode.Auto")
@Config
public class RightSideAuto_4Specimen extends LinearOpMode {

    public static double highbar_x_coordinate = PointToDrive.highbar_x_coordinate;
    public static double highbar_y_coordinate = PointToDrive.highbar_y_coordinate;
    public static double highbar_x_coordinate2 = PointToDrive.highbar_x_coordinate2;
    public static double highbar_x_coordinate3 = PointToDrive.highbar_x_coordinate3;
    public static double specimen_pickup_x_coordinate = PointToDrive.specimen_pickup_x_coordinate;
    public static double specimen_pickup_y_coordinate = PointToDrive.specimen_pickup_y_coordinate;
    public static double first_sample_pickup_x_coordinate = PointToDrive.first_Color_sample_pickup_x_coordinate;
    public static double first_sample_pickup_y_coordinate = PointToDrive.first_Color_sample_pickup_y_coordinate;
    public static double second_sample_pickup_x_coordinate = PointToDrive.second_Color_sample_pickup_x_coordinate;
    public static double second_sample_pickup_y_coordinate = PointToDrive.second_Color_sample_pickup_y_coordinate;
    public static double clawOpenTimer = 0.2;
    public static double waitTimer = 0.25;
    public static double vSlideWaitTimer = 0.5;
    public static double hSlideWaitTimer =  0.35;
    public static double wristWaitTimer = 0.25;

    private ElapsedTime timer = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    VerticalSlide vSlides;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot.init(hardwareMap);
        vSlides = new VerticalSlide(robot);

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

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                //extend slides to specimen scoring position
                .addTemporalMarker(() -> {
                    vSlides.slidesMove(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    depositHookPosition();
                })
                //drive to bar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
                .build();

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //extend slides to specimen scoring position
                .addTemporalMarker(() -> {
                    vSlides.slidesMove(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })

                //drive to bar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
                //Hook 1st specimen
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.08, () -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.1)

        /** 2nd Segment --> grab 1st red samples for specimen*/
            /**Move to 1st sample*/
                .lineToLinearHeading(new Pose2d(first_sample_pickup_x_coordinate, first_sample_pickup_y_coordinate, Math.toRadians(90)))
                ///drop slides and put arm back to transfer position while robot moving
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    vSlides.slidesMoveDown(RobotActionConfig.deposit_Slide_Down_Pos, 0.8); //Move Slides Down
                })
                ///Extend slide to grab 1st sample, -0.5 sec is the timefor slide extension and intake arm lower down.
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                /// Robot stationed.
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                    vSlides.Slides_Stop();
                })
                .waitSeconds(0.1) /// make robot stable for sample pick.
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2) /** ! HAS to pause for 0.2s for secure pick up.*/

                ///Transfer 1st sample
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

                ///Drop sample into OB Zone
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(1.3)

            /**Move to 2nd sample*/
                .lineToLinearHeading(new Pose2d(second_sample_pickup_x_coordinate, second_sample_pickup_y_coordinate, Math.toRadians(90)))
                ///Return to transfer position while robot is moving
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })

                ///Grab 2nd sample
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .waitSeconds(0.25+hSlideWaitTimer) // time is 0.6 sec in total.
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)  /** ! HAS to pause for 0.2sec for secure pick up.*/
                ///Transfer 2nd sample
                .addTemporalMarker(() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .waitSeconds(wristWaitTimer)        // 0.25 seconds for intake arm back to transfer
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

                ///Drop sample into OB Zone
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(1.3)

        /** 3rd Segment --> move to Specimen pick up position*/
            /// 3.1 ---->  Move to 2nd specimen on ground
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                ///Return to transfer position while robot is moving
                .UNSTABLE_addTemporalMarkerOffset(-0.5, this::depositTransferPosition)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                /// ----> Grab 2nd specimen from ground
                .UNSTABLE_addTemporalMarkerOffset(0, this::intakeSpecimenPickReady)
                .UNSTABLE_addTemporalMarkerOffset(0.5, this::intakeSpecimenPick)
                .waitSeconds(0.45+hSlideWaitTimer)  ///0.8 - 0.5 = 0.3 sec to give human player time to response.
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2) /** ! HAS to be 0.2 sec for secured grab the specimen*/
                ///Transfer 2ND SPECIMEN - RETRACT the intake arm and retract the intake slide and 0.25 sec later to retract slide
                .addTemporalMarker(() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .waitSeconds(wristWaitTimer)                // 0.25 secs
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
                .waitSeconds(0.1)
            /// 3.2 ----> move to highbar spot for 2nd specimen scoring
                    //  ----> extend vertical slides to scoring position - 2nd specimen
                .addTemporalMarker(()->{extendDepositSysScoring();})                                   // extend vertical slide, deposit arm and deposit wrist
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate2, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                // ----> open claw and flat wrist - scored 2nd specimen
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.08, () -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.1)
            /// move to 3rd specimen to pick up
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                //DEPOSIT ARM BACK TO TRANSFER POSITION & DROP SLIDES
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    vSlides.slidesMoveDown(RobotActionConfig.deposit_Slide_Down_Pos,0.8);
                    depositTransferPosition();
                })
                //STATIONED the Robot
                 .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                 drive.setDrivePower(new Pose2d(0, 0, 0));
                 })
                // PICK the 3rd specimen - EXTEND THE SLIDE.
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                // FULL EXTEND OUT TO PICK - 0.5 SEC AFTER ALLOWING HUMAN PLAYER TO RESPONSE
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {intakeSpecimenPick();})
                .waitSeconds(0.45+hSlideWaitTimer) // 0.8 - 0.5 = 0.3s
            ///GRAB THE 3rd specimen
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)         /** !HAS TO BE 0.2 SEC TO GRABE the specimen*/
            ///TRANSFER 3RD SPECIMEN
                .addTemporalMarker(() -> {
                 robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                 robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                 robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .waitSeconds(wristWaitTimer)                // 0.25 secs
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
                .waitSeconds(0.1)
            /// ----> MOVE TO HIGHBAR FOR 3RD SPECIMEN SCORING
                 .addTemporalMarker(() -> {
                     vSlides.slidesMove(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                     robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
                     robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                     robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                 })
                 .lineToLinearHeading(new Pose2d(highbar_x_coordinate3, highbar_y_coordinate, Math.toRadians(-90)))
                 .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                 })
                 // ----> open claw and flat wrist - scored 2nd specimen
                 .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                 })
                 .UNSTABLE_addTemporalMarkerOffset(0.08, () -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                 })
                 .waitSeconds(0.1)
            /// ----> move to 4th specimen pick up position
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> {
                    //vSlides.slidesMoveDown(RobotActionConfig.deposit_Slide_Down_Pos,0.8);
                    //robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    //robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    depositSysRetract();
                })
                /// ---> extend slide and open claw to pick up
                .UNSTABLE_addTemporalMarkerOffset(0.0,() -> {
                    //robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    //robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    //robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    //robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    //robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    //robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                    intakeSpecimenPickReady();
                })
                /// ---> extend slide to pick
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    //robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    //robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    //robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    intakeSpecimenPick();
                })
                .waitSeconds(0.45+hSlideWaitTimer)
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
                .waitSeconds(1.1)
                //extend slides to scoring position
                .addTemporalMarker(() -> {
                    vSlides.slidesMove(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                //move to highbar 4th Specimen
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //open claw and flat wrist
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(0.08)
                .addTemporalMarker(() -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                /** ----> Extend slides to OB ZONE*/
                .UNSTABLE_addTemporalMarkerOffset(-0.5,() -> {
                        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                        vSlides.slidesMoveDown(RobotActionConfig.deposit_Slide_Down_Pos, 0.8);
                        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                })
                .build();

        waitForStart();
        //set the timer
        timer.reset();
        long currentTime = System.currentTimeMillis();
        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq2);

        PoseStorage.currentPose = drive.getPoseEstimate();
        while (opModeIsActive()) {
            telemetry.addData("Slides Left Position: ", robot.liftMotorLeft.getCurrentPosition());
            telemetry.addData("Slides Right Position: ", robot.liftMotorRight.getCurrentPosition());
            telemetry.update();
        }
    }

    /** Slides subclass for vertical slides moving helper method.*/
    class VerticalSlide {
        RobotHardware robot;
        private VerticalSlide(RobotHardware robot){
            this.robot = robot;
        }
        private void Slides_Stop() {
            robot.liftMotorLeft.setPower(0);
            robot.liftMotorRight.setPower(0);
        }

        private void slidesMove(double targetPosition, double speed) {
            int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
            robot.liftMotorLeft.setTargetPosition(targetTick);
            robot.liftMotorRight.setTargetPosition(targetTick);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }

        private void slidesMoveDown(double targetPosition, double speed) {
            int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
            robot.liftMotorLeft.setTargetPosition(targetTick);
            robot.liftMotorRight.setTargetPosition(targetTick);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);

            long startTime = System.currentTimeMillis();  // Track start time
            long timeout = 2500;  // Set a timeout of 5 seconds to avoid getting stuck

            while (opModeIsActive() && (robot.liftMotorLeft.isBusy() || robot.liftMotorRight.isBusy())) {
                if (isLimitSwitchPressedFor200ms()) {  // Pass actual time
                    Slides_Stop();
                    break;
                }
            }
        }
    }

    /**HELPER METHODS*/
    /** limit switch helper
    private boolean lSisPressed(long currentTime) {
        if (currentTime - RobotActionConfig.lastPressedTime > RobotActionConfig.debounceDelay) {
            RobotActionConfig.lastPressedTime = currentTime;
            return robot.limitSwitch.getState();  // Invert if switch is normally open
        }
        return false;
    }*/
    private long limitSwitchPressStartTime = 0;

    // Original method that returns the current state of the limit switch.
    private boolean lSisPressed() {
        return robot.limitSwitch.getState();  // Invert if switch is normally open
    }

    /**
     * Returns true only if the limit switch has been pressed continuously for at least 200ms.
     */
    private boolean isLimitSwitchPressedFor200ms() {
        if (lSisPressed()) {
            // If the switch is pressed and we haven't recorded the start time yet,
            // record the current time.
            if (limitSwitchPressStartTime == 0) {
                limitSwitchPressStartTime = System.currentTimeMillis();
            }
            // Check if 200ms have elapsed.
            if (System.currentTimeMillis() - limitSwitchPressStartTime >= 100) {
                return true;
            }
        } else {
            // If the switch is not pressed, reset the timer.
            limitSwitchPressStartTime = 0;
        }
        return false;
    }

    //depositTransferPosition
    private void depositTransferPosition(){
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
    }
    //depositHookPosition
    private void depositHookPosition(){
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
    }

    //DEPOSIT SYSTEM Action Helper
    public void depositSysRetract() {
        depositTransferPosition();
        vSlides.slidesMoveDown(RobotActionConfig.deposit_Slide_Down_Pos, 0.8);
    }

    ///VERTICAL SLIDES FEATURE MOVE
    public void extendDepositSysScoring(){
        vSlides.slidesMove(RobotActionConfig.deposit_Slide_Highbar_Pos, 1);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
        depositHookPosition();
    }

    public void intakeSpecimenPickReady(){
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
    }

    public void intakeSpecimenPick(){
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
    }

    private void executeTransfer(){
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);

        waitFor(0.25);                // 0.25 secs;
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        waitFor(0.2);                // 0.25 secs;
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        waitFor(0.2);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        waitFor(0.1);
    }

    private void waitFor(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) {
            // Optionally call idle() or update telemetry here.
        }
    }
}