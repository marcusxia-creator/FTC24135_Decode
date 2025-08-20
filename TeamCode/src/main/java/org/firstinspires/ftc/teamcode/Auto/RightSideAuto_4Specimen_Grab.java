

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

@Autonomous(name="RightSideAuto_4Specimen_Wall_Grab", group="org.firstinspires.ftc.teamcode.Auto")
@Config
public class RightSideAuto_4Specimen_Grab extends LinearOpMode {

    public static double highbar_x_coordinate = -5;
    public static double highbar_y_coordinate = -31;
    public static double highbar_x_coordinate2 = -3;
    public static double highbar_x_coordinate3 = -1;
    public static double highbar_x_coordinate4 = 0;
    public static double specimen_pickup_x_coordinate = PointToDrive.specimen_pickup_x_coordinate;
    public static double specimen_pickup_y_coordinate = PointToDrive.specimen_pickup_y_coordinate;
    public static double bar_out_point_1_X = 24;
    public static double bar_out_point_1_Y = -55;

    public static double first_sample_point_1_X = 29.5;
    public static double first_sample_point_1_Y = -46; //heading = 45
    public static double first_sample_point_1_heading = 35;

    public static double first_sample_point_2_X = 29.5;
    public static double first_sample_point_2_Y = -50; //heading = -45

    public static double second_sample_point_1_X = 38;
    public static double second_sample_point_1_Y = -42; //heading = 45
    public static double second_sample_point_1_heading = -31; //heading = -45

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


        Pose2d startPose = new Pose2d(7.5, -64, Math.toRadians(90));
        drive.pinpointSetPose(startPose);
        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(90)))
                ///-----------------------------------------------------------------------
                /** --> push ground sample for specimen*/

                .lineToLinearHeading(new Pose2d(first_sample_point_1_X, first_sample_point_1_Y, Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })

                .UNSTABLE_addTemporalMarkerOffset(-0.4,()->{
                    Slides_Move(RobotActionConfig.deposit_Slide_Pick_Rear_Pos,1.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3,()->{
                    Depo_Pick();
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid+0.15);
                    robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.25,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    Intake_Grab();
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    Intake_Pick();
                })
                .waitSeconds(0.45)
                .addTemporalMarker(()->{
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Pick-0.02);
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
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                })
                .waitSeconds(0.20)
                .addTemporalMarker(()->{
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.20)
                /**transition to scoring*/
                 .lineToLinearHeading(new Pose2d(first_sample_point_2_X, first_sample_point_2_Y, Math.toRadians(-45)))
                 .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                 drive.setDrivePower(new Pose2d(0, 0, 0));})
                 .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                 robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                 })
                 .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                 robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Side_Drop);
                 robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Side_Drop);
                 robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Side_Drop);
                 robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                 robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                 robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                 robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                 robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                 })
                 .waitSeconds(0.3)
                 ///-----------------------------------------------------------------------
                 ///score 2nd sample
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));})
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos,1.0);
                    Depo_Hook();
                })
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate2,highbar_y_coordinate, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Score_Pos,0.5);
                })
                //open claw
                .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{
                    Slides_Move(RobotActionConfig.deposit_Slide_Pick_Rear_Pos,0.7);
                    Depo_Pick();
                })
                .waitSeconds(1.0)
                 ///score 3rd sample
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));})
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos,0.9);
                    Depo_Hook();
                })
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate3,highbar_y_coordinate, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));})
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Score_Pos,0.5);
                })
                //open claw
                .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{
                    Slides_Move(RobotActionConfig.deposit_Slide_Pick_Rear_Pos,0.7);
                    Depo_Pick();
                })
                .waitSeconds(1.0)
                 ///score 4th sample
                 .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(90)))
                 .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                 drive.setDrivePower(new Pose2d(0, 0, 0));})
                 .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                 robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                 })
                 .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                 Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos,0.9);
                 Depo_Hook();
                 })
                 .waitSeconds(0.3)
                 .lineToLinearHeading(new Pose2d(highbar_x_coordinate4,highbar_y_coordinate, Math.toRadians(90)))
                 .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                 drive.setDrivePower(new Pose2d(0, 0, 0));})
                 .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                 Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Score_Pos,0.5);
                 })
                 //open claw
                 .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                 robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                 })
                 .UNSTABLE_addTemporalMarkerOffset(0.9,()->{
                 Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos,0.7);
                 Depo_Transfer();
                 })
                 .waitSeconds(1.0)
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(90)))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
        while (opModeIsActive()) {
            telemetry.addData("Slides Position", robot.liftMotorLeft.getCurrentPosition());
            //telemetry.addData("Battery Voltage", robot.voltageSensor);
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

    private void Depo_Hook(){
        robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
        robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
    }
    private void Depo_Transfer(){
        robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
    }
    private void Depo_Pick(){
        robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Pick);
        robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Pick);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Pick);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
    }
    private void Intake_Grab(){
        robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);
        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
    }
    private void Intake_Pick(){
        robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);
        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
    }
    private void Intake_Side_Drop(){
        robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Side_Drop);
        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Side_Drop);
        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Wrist_Side_Drop);
    }
    private void Intake_Slide_Extend(){
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
    }
    private void Intake_Slide_Retract(){
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
    }
}
