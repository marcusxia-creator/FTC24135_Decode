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


@Autonomous(name="LeftSideAuto_4_Sample", group="org.firstinspires.ftc.teamcode.Auto")
@Config
public class LeftSideAuto_4Sample_Provincial extends LinearOpMode {

    RobotHardware robot = new RobotHardware();                          //RobotHardware is from TeleOps.
    static final double COUNTS_PER_MOTOR_GOBILDA_435 = 384.5;
    static final double COUNTS_PER_MOTOR_GOBILDA_312 = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.5; //24:16 Motor:Wheel
    static final double WHEEL_DIAMETER_MM = 96; // Wheel diameter mm
    static final double COUNTS_PER_MM_Drive = (COUNTS_PER_MOTOR_GOBILDA_435 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);


    // Timer Configuration
    public static double dumpTime = 0.5; // deposit time need to rotate deposit arm then open claw

    //movement positions
    public static double basket_x_coordinate = -58.5;    //55.5 for Competition Basket, 58.5 for Home Basket
    public static double basket_y_coordinate = -58.5;
    public static double first_sample_x_coordinate = -49.5;
    public static double first_sample_y_coordinate = -53;
    public static double second_sample_x_coordinate = -60;
    public static double second_sample_y_coordinate = -53;
    public static double third_sample_x_coordinate = -47;
    public static double third_sample_y_coordinate = -41;
    public static double third_sample_heading = 150;

    public static double rightPark_x_coordiante = -21.5;
    public static double rightPark_y_coordiante = 6.5;
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
                ///Score Preload Sample
                .addTemporalMarker(()-> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbasket_Pos, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1,()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);
                })
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                //run to basket
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})                                                                                                    // wait slide riseup
                .waitSeconds(0.3)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})                          // open claw
                .waitSeconds(0.4)                                                                                                           // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);                                          // at global time 1.5 second mark to back to transfer position
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })                    //wrist transfer
                .lineToLinearHeading(new Pose2d(first_sample_x_coordinate,first_sample_y_coordinate,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-0.9,()->{Slides_MoveDown(RobotActionConfig.deposit_Slide_Down_Pos,0.9);}) // Lower slides dist in mm
                .waitSeconds(0.35)
                .addTemporalMarker(()->{
                    Slides_Stop();
                })
                /** move to 1st sample*/
                /** pick 1st sample*/
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
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
                .waitSeconds(1.5)
                ///Score 1st Sample
                .addTemporalMarker(()-> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbasket_Pos, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1,()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);
                })
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                //run to basket
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})                      // wait slide riseup
                .waitSeconds(0.5)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})                          // open claw
                .waitSeconds(0.5)                                                                                                           // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);                                              // at global time 1.5 second mark to back to transfer position
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .lineToLinearHeading(new Pose2d(second_sample_x_coordinate,second_sample_y_coordinate,Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{Slides_MoveDown(RobotActionConfig.deposit_Slide_Down_Pos,0.9);})    // Lower slides dist in mm
                .waitSeconds(0.45)
                .addTemporalMarker(()->{
                    Slides_Stop();
                })
                /** move to 2nd sample*/
                /** pick 2nd sample*/
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
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
                .waitSeconds(1.5)
                .addTemporalMarker(()-> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbasket_Pos, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1,()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);
                })
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                //run to basket
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})// wait slide riseup
                .waitSeconds(0.8)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})                          // open claw
                .waitSeconds(0.5)                                                                                                           // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);                                              // at global time 1.5 second mark to back to transfer position
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Slides_MoveDown(RobotActionConfig.deposit_Slide_Down_Pos,0.9);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid - 0.2);
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Pick - 0.05);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick - 0.05);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })  // Lower slides dist in mm
                .lineToLinearHeading(new Pose2d(third_sample_x_coordinate,third_sample_y_coordinate,Math.toRadians(third_sample_heading)))
                .UNSTABLE_addTemporalMarkerOffset(-0.05,()->{Slides_Stop();})
                .addTemporalMarker(()->{
                    drive.setDrivePower(new Pose2d(0,0,0));
                })
                /** pick 3rd sample*/
                .addTemporalMarker(()->{
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Grab);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Grab);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                })
                //0.15 sec later to retract slide
                .UNSTABLE_addTemporalMarkerOffset(0.15,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(()-> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbasket_Pos, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1,()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);
                })
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))                                //run to basket
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})// wait slide riseup
                .waitSeconds(0.5)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})                          // open claw
                .waitSeconds(0.5)                                                                                                           // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);                                              // at global time 1.5 second mark to back to transfer position
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_MoveDown(RobotActionConfig.deposit_Slide_Down_Pos,0.9);})  // Lower slides dist in mm
                .waitSeconds(1.25)
                .addTemporalMarker(()->{
                    Slides_Stop();
                })
                .lineToLinearHeading(new Pose2d(rightPark_x_coordiante,rightPark_y_coordiante,Math.toRadians(rightPark_heading)))
                .UNSTABLE_addTemporalMarkerOffset(-0.7,()->{
                    robot.depositArmServo.setPosition(0.4);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(2)
                .build();

        waitForStart();
        timer.reset();
        long currentTime = System.currentTimeMillis();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
        /// update the currentPose
        PoseStorage.currentPose = drive.getPoseEstimate();
        while (opModeIsActive()) {
            telemetry.addData("Slides Left Position: ", robot.liftMotorLeft.getCurrentPosition());
            telemetry.addData("Slides Right Position: ", robot.liftMotorRight.getCurrentPosition());
            telemetry.update();
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