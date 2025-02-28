package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;


@Autonomous(name="LeftSideAuto_League", group="org.firstinspires.ftc.teamcode.Auto")
@Config
public class LeftSideAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware();                          //RobotHardware is from TeleOps.
    static final double COUNTS_PER_MOTOR_GOBILDA_435 = 384.5;
    static final double COUNTS_PER_MOTOR_GOBILDA_312 = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.5; //24:16 Motor:Wheel
    static final double WHEEL_DIAMETER_MM = 96; // Wheel diameter mm
    static final double COUNTS_PER_MM_Drive = (COUNTS_PER_MOTOR_GOBILDA_435 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);


    // Timer Configuration
    public static double dumpTime = 0.5; // deposit time need to rotate deposit arm then open claw

    //movement positions
    public static double basket_x_coordinate = -57;
    public static double basket_y_coordinate = -55;
    public static double first_sample_x_coordinate = -50;
    public static double first_sample_y_coordinate = -35;
    public static double second_sample_x_coordinate = -61.8;
    public static double second_sample_y_coordinate = -34.5;
    public static double third_sample_x_coordinate = -58;
    public static double third_sample_y_coordinate = -37;
    public static double third_sample_heading = 114;

    public static double rightPark_x_coordiante = -20;
    public static double rightPark_y_coordiante = 8;
    public static double rightPark_heading = 180;


    private ElapsedTime timer = new ElapsedTime();

    float timer_log;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        robot.init(hardwareMap);

        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);

        Pose2d startPose = new Pose2d(-40, -64.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                //run to basket
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{Slides_Move(RobotActionConfig.deposit_Slide_Highbasket_Pos,1);})     // targetPosition in cm - raise slides
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump_Prep);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(1.5)       //wait time to rise slides.                                                                                                    // wait slide riseup
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);})                            // Arm dump
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);})                        // Wrist dump
                .waitSeconds(0.2)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})                          // open claw
                .waitSeconds(0.2)                                                                                                           // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);})                        // at global time 1.5 second mark to back to transfer position
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);})                    //wrist transfer
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{Slides_MoveDown(RobotActionConfig.deposit_Slide_Down_Pos,0.9);})                                          // Lower slides dist in mm
                .waitSeconds(1.25)       // wait time to lower slides.
                .addTemporalMarker(() -> {
                    Slides_Stop();
                })
                .waitSeconds(0.25)      // wait 0.25 seconds to stationary the robot --- may not need.
                /** move to 1st sample*/
                .lineToLinearHeading(new Pose2d(first_sample_x_coordinate,first_sample_y_coordinate,Math.toRadians(90)))                    //move to 1st sample
                /** pick 1st sample*/
                .addTemporalMarker(()->{robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick +0.02);})
                .addTemporalMarker(()->{robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick +0.02);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);})
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.5)
                /**transfer 1st sample*/
                .addTemporalMarker(()->{
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick -0.03);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick -0.03);
                    robot.intakeRightSlideServo.setPosition(0.1);
                    robot.intakeLeftSlideServo.setPosition(0.1);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);})
                .addTemporalMarker(()->{
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract+0.01);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract+0.01);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                                       // move to basket
                /** drop 1st sample*/
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})                                             //stop
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump_Prep);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_Move(RobotActionConfig.deposit_Slide_Highbasket_Pos,1);})                                                                // slide raise
                .waitSeconds(1.5) //wait time to rise the slide                                                                                                                                 // wait slides
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);})                                                  //Arm dump start after wait seconds
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);})                                              //wrist dump
                .waitSeconds(0.2)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})
                .waitSeconds(0.3)                                                                                                                                // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);})                                             // at global time 1.5 second mark to back to transfer position
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos,0.9);})                                                               // at global time 1.5 second mark to back to lower slides.
                .waitSeconds(1.5) //wait time to lower the slide
                .addTemporalMarker(() -> {
                    Slides_Stop();
                })
                .waitSeconds(0.5)
                /** move to 2nd sample */
                .lineToLinearHeading(new Pose2d(second_sample_x_coordinate,second_sample_y_coordinate,Math.toRadians(90)))
                /** pick 2nd sample*/
                .addTemporalMarker(()->{robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick +0.02);})
                .addTemporalMarker(()->{robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick +0.02);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);})
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.5)
                /**transfer 2nd sample*/
                .addTemporalMarker(()->{
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick -0.03);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick -0.03);
                    robot.intakeRightSlideServo.setPosition(0.1);
                    robot.intakeLeftSlideServo.setPosition(0.1);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);})
                .addTemporalMarker(()->{
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract+0.01);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract+0.01);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                                  // move to basket
                /** drop 2nd sample*/
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})                                        //stop
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump_Prep);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_Move(RobotActionConfig.deposit_Slide_Hang_Pos,1);})                                                           // slide raise
                .waitSeconds(1.5) // wait slide to rise up                                                                                                                            // wait slides
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);})                                             //Arm dump start after wait seconds
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);})                                         //wrist dump
                .waitSeconds(0.2)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})
                .waitSeconds(0.4)                                                                                                                            // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);})                                         // at global time 1.5 second mark to back to transfer position
                        .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);})
                        .UNSTABLE_addTemporalMarkerOffset(0.2,()->{Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos,0.9);})                                                   // at global time 1.5 second mark to back to lower slides.
                .waitSeconds(1.9) // wait slide to lower down up
                .addTemporalMarker(() -> {
                    Slides_Stop();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(rightPark_x_coordiante,rightPark_y_coordiante,Math.toRadians(rightPark_heading)))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    robot.depositArmServo.setPosition(0.4);
                    robot.depositWristServo.setPosition(0.05);
                })
                .build();

        waitForStart();
        timer.reset();
        long currentTime = System.currentTimeMillis();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }

    }

    private void Slides_Move(int targetPosition, double speed) {
        // targetPosition in mm - raise slides
        int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
        robot.liftMotorLeft.setTargetPosition(targetTick);
        robot.liftMotorRight.setTargetPosition(targetTick);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
        /**
        while ((robot.liftMotorLeft.isBusy() || robot.liftMotorRight.isBusy()) && opModeIsActive()){
            if(LSisPressed(200)|| IsLiftDownAtPosition(targetPosition) ){
                Slides_Stop();
                break;
            }
        }
         */
    }
    private void Slides_MoveDown(int targetPosition, double speed) {
        // targetPosition in cm - raise slides
        int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
        robot.liftMotorLeft.setTargetPosition(targetTick);
        robot.liftMotorRight.setTargetPosition(targetTick);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
        while ((robot.liftMotorLeft.isBusy() || robot.liftMotorRight.isBusy()) && opModeIsActive()){
            if(LSisPressed(200)|| IsLiftDownAtPosition(targetPosition) ){
                Slides_Stop();
            }
        }
    }

    private void Slides_Stop(){
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
    }

    //Limit switch state
        //Limit switch state
    private boolean LSisPressed(long currentTime) {
        if(currentTime - RobotActionConfig.lastPressedTime > RobotActionConfig.debounceDelay){
            RobotActionConfig.lastPressedTime = currentTime;
            return robot.limitSwitch.getState();
        } else{
            return false;
        }
    }
    private boolean IsLiftDownAtPosition(int targetPosition) {
        int targetTicks = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold/2 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetTicks) < RobotActionConfig.slideTickThreshold/2;
    }
}