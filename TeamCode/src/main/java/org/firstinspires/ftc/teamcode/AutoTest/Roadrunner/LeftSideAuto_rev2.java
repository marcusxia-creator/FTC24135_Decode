package org.firstinspires.ftc.teamcode.AutoTest.Roadrunner;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;


@Autonomous(name="LeftSideAuto_test v2", group="org.firstinspires.ftc.teamcode.AutoTest.Roadrunner")
@Config
public class LeftSideAuto_rev2 extends LinearOpMode {

    RobotHardware robot = new RobotHardware();                          //RobotHardware is from TeleOps.
    static final double COUNTS_PER_MOTOR_GOBILDA_435 = 384.5;
    static final double COUNTS_PER_MOTOR_GOBILDA_312 = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.5; //24:16 Motor:Wheel
    static final double WHEEL_DIAMETER_MM = 96; // Wheel diameter mm
    static final double COUNTS_PER_MM_Drive = (COUNTS_PER_MOTOR_GOBILDA_435 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
    static final double COUNTS_PER_CM_Slides = COUNTS_PER_MOTOR_GOBILDA_312 / 38.2; //Ticks Per Rotation * Pulley Circumference

    // Timer Configuration
    public static double dumpTime = 0.5; // deposit time need to rotate deposit arm then open claw

    //movement positions
    public static double basket_x_coordinate = -59;
    public static double basket_y_coordinate = -59;
    public static double first_sample_x_coordinate = -48.7;
    public static double first_sample_y_coordinate = -36.5;
    public static double second_sample_x_coordinate = -60.5;
    public static double second_sample_y_coordinate = -36.5;
    public static double third_sample_x_coordinate = -58;
    public static double third_sample_y_coordinate = -36.5;
    public static double third_sample_heading = 114;

    public static double rightPark_x_coordiante = -18;
    public static double rightPark_y_coordiante = -3;
    public static double rightPark_heading = 180;


    public ElapsedTime timer = new ElapsedTime();

    float timer_log;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.pinpointresetIMU();

        robot.init(hardwareMap);

        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);

        Pose2d startPose = new Pose2d(-40, -64.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                     //run to basket
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_Move(3226,0.9);})                                               //raise slides
                .waitSeconds(1.8)                                                                                                                // wait slide rise-up
                /** drop the sample from pre-set*/
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);})       // while slide riseup, run deposit arm to dump
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);})   // while slide riseup, run deposit wrist to dump
                //.addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);})                // Arm dump
                //.addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);})            // Wrist dump
                .waitSeconds(0.5)                                                                                                               // wait 0.5 to open claw
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})                              // open claw
                .waitSeconds(0.5)                                                                                                               // 0.5 wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);})                            // at global time 2.8s mark to back to transfer position
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);})                        // wrist back to transfer position
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_Move(50,0.9);})                                                // Lower slides while arm and wrist back to transfer
                .waitSeconds(1.8)                                                                                                               // wait 1.8s to lower slides.(4.6s global time)

                /** move to 1st sample*/
                .lineToLinearHeading(new Pose2d(first_sample_x_coordinate,first_sample_y_coordinate,Math.toRadians(90)))        //move to 1st sample
                /** pick 1st sample*/
                .addTemporalMarker(()->{robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);})                              // intake arm drop to pick position
                .addTemporalMarker(()->{robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);})                             // intake arm
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{robot.depositWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);})    //0.2s after arm move then wrist drop to pick position
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})                                // intake claw open when move arm.
                .waitSeconds(0.7)                                                                                                               // move 0.7s - waiting intake arm and wrist lower to pick position.
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})                               // close intake claw
                .waitSeconds(0.5)                                                                                                               // wait 0.5s
                /**transfer 1st sample*/
                .addTemporalMarker(()->{robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);})                          // run both intake arm and wrist back to transfer
                .addTemporalMarker(()->{robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);})
                .addTemporalMarker(()->{robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);})
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                    // move to basket -- which can be move up to allow run both arm and
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})    // offset -0.5s to close the deposit claw while moving the robot.
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})       // offset -0.3s to open intake claw while moving the robot.

                /** drop 1st sample*/
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})          //stop
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_Move(3226,0.9);})                             // slide raise
                .waitSeconds(1.8)                                                                                                                // wait slide rise-up
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);})       // while slide riseup, run deposit arm to dump
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);})   // while slide riseup, run deposit wrist to dump
                .waitSeconds(0.5)                                                                                                               // wait 0.5 to open claw
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})                              // open claw
                .waitSeconds(0.5)                                                                                                               // 0.5 wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);})                            // at global time 2.8s mark to back to transfer position
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);})                        // wrist back to transfer position
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_Move(50,0.9);})                                                // Lower slides while arm and wrist back to transfer
                .waitSeconds(1.8)                                                                                                               // wait for sldie back to bottom

                /** move to 2nd sample */
                .lineToLinearHeading(new Pose2d(second_sample_x_coordinate,second_sample_y_coordinate,Math.toRadians(90)))      // move the 2nd sample
                /** pick 2nd sample*/
                .addTemporalMarker(()->{robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);})              // move intake arm to pick pos
                .addTemporalMarker(()->{robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);})
                .addTemporalMarker(()->{robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);})              //move intake waist to pick pos at the same time
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})                // open the intake claw while turning intake arm and waist
                .waitSeconds(0.5)                                                                                               // wait 0.5 s
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})               // close claw
                .waitSeconds(0.5)
                /**transfer 2nd sample*/
                .addTemporalMarker(()->{robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);})
                .addTemporalMarker(()->{robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);})
                .addTemporalMarker(()->{robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);})
                .UNSTABLE_addTemporalMarkerOffset(1.1,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})   //wait 1.1 s all intake arm transfer and waisty transfer
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})      // wait 0.1 s open the intake claw
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                   // move to basket
                /** drop 2nd sample*/
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})                      //stop
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_Move(3226,0.9);})                                          // slide raise
                .waitSeconds(2.0)                                                                                                           // wait slides
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);})                              //Arm dump start after wait seconds
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);})                          //wrist dump
                .waitSeconds(0.7)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})
                .waitSeconds(0.5)                                                                                          // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);})                      // at global time 1.5 second mark to back to transfer position
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{Slides_Move(50,0.9);})                             // at global time 1.5 second mark to back to lower slides.
                .waitSeconds(2.5)
                .lineToLinearHeading(new Pose2d(rightPark_x_coordiante,rightPark_y_coordiante,Math.toRadians(rightPark_heading)))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);})
                .build();

        waitForStart();
        timer.reset();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }

    }

    private void Slides_Move(int dist, double speed) {
        robot.liftMotorLeft.setTargetPosition(dist);
        robot.liftMotorRight.setTargetPosition(dist);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
        while (opModeIsActive() && (robot.liftMotorLeft.isBusy() && robot.liftMotorRight.isBusy())) {

        }
        sleep(200);
    }
}
