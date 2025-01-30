package org.firstinspires.ftc.teamcode.AutoTest.Roadrunner;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;


@Autonomous(name="RightSideAuto", group="org.firstinspires.ftc.teamcode.AutoTest.Roadrunner")
@Config
@Disabled
public class RightSideAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    static final double COUNTS_PER_MOTOR_GOBILDA_435 = 384.5;
    static final double COUNTS_PER_MOTOR_GOBILDA_312 = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.5; //24:16 Motor:Wheel
    static final double WHEEL_DIAMETER_MM = 96; // Wheel diameter mm
    static final double COUNTS_PER_MM_Drive = (COUNTS_PER_MOTOR_GOBILDA_435 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
    static final double COUNTS_PER_CM_Slides = COUNTS_PER_MOTOR_GOBILDA_312 / 38.2; //Ticks Per Rotation * Pulley Circumference



    //movement positions
    public static double hook_x_coordinate = -59;
    public static double hook_y_coordinate = -59;
    public static double hook_x2_coordinate = -39;
    public static double hook_y2_coordinate = -39;
    public static double hook_heading = 0;
    public static double specimen_x_coordinate = -48;
    public static double specimen_y_coordinate = -38;
    public static double pick_heading = 180;
    public static double mid_x_coordinate = -58;
    public static double mid_y_coordinate = -48;
    public static double first_sample_x_coordinate = -58;
    public static double first_sample_y_coordinate = -48;
    public static double second_sample_x_coordinate = -58;
    public static double second_sample_y_coordinate = -48;
    public static double third_sample_x_coordinate = -58;
    public static double third_sample_y_coordinate = -48;
    public static double third_sample_heading = 114;

    public ElapsedTime timer = new ElapsedTime();

    float timer_log;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        robot.init(hardwareMap);

        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);

        Pose2d startPose = new Pose2d(-40, -64.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(hook_x_coordinate, hook_y_coordinate, Math.toRadians(hook_heading)))
                .addTemporalMarker(() -> {RunHookPosition();})
                .lineToLinearHeading(new Pose2d(hook_x_coordinate, hook_y_coordinate+20, Math.toRadians(hook_heading)))
                .addTemporalMarker(() -> {Release();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(hook_x_coordinate, hook_y_coordinate, Math.toRadians(hook_heading)))
                .lineToLinearHeading(new Pose2d(mid_x_coordinate, mid_y_coordinate, Math.toRadians(pick_heading)))
                //
                .lineToLinearHeading(new Pose2d(mid_x_coordinate, mid_y_coordinate+100, Math.toRadians(pick_heading)))
                .lineToLinearHeading(new Pose2d(first_sample_x_coordinate, first_sample_y_coordinate, Math.toRadians(pick_heading)))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(specimen_x_coordinate, specimen_y_coordinate, Math.toRadians(pick_heading)))
                .lineToLinearHeading(new Pose2d(second_sample_x_coordinate, second_sample_y_coordinate, Math.toRadians(pick_heading)))
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(specimen_x_coordinate, specimen_y_coordinate, Math.toRadians(pick_heading)))
                .lineToLinearHeading(new Pose2d(third_sample_x_coordinate, third_sample_y_coordinate, Math.toRadians(pick_heading)))
                .waitSeconds(0.2)
                //
                .lineToLinearHeading(new Pose2d(specimen_x_coordinate, specimen_y_coordinate, Math.toRadians(pick_heading)))
                .addTemporalMarker(() -> {RunSpecimenPosition();})
                .lineToLinearHeading(new Pose2d(specimen_x_coordinate, specimen_y_coordinate-20, Math.toRadians(pick_heading)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(hook_x_coordinate, hook_y_coordinate, Math.toRadians(hook_heading)))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {RunHookPosition();})
                .lineToLinearHeading(new Pose2d(hook_x_coordinate, hook_y_coordinate+20, Math.toRadians(hook_heading)))
                .addTemporalMarker(() -> {Release();})
                //
                .lineToLinearHeading(new Pose2d(specimen_x_coordinate, specimen_y_coordinate, Math.toRadians(pick_heading)))
                .addTemporalMarker(() -> {RunSpecimenPosition();})
                .lineToLinearHeading(new Pose2d(specimen_x_coordinate, specimen_y_coordinate-20, Math.toRadians(pick_heading)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(hook_x_coordinate, hook_y_coordinate, Math.toRadians(hook_heading)))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {RunHookPosition();})
                .lineToLinearHeading(new Pose2d(hook_x_coordinate, hook_y_coordinate+20, Math.toRadians(hook_heading)))
                .addTemporalMarker(() -> {Release();})
                //
                .lineToLinearHeading(new Pose2d(specimen_x_coordinate, specimen_y_coordinate, Math.toRadians(pick_heading)))
                .addTemporalMarker(() -> {RunSpecimenPosition();})
                .lineToLinearHeading(new Pose2d(specimen_x_coordinate, specimen_y_coordinate-20, Math.toRadians(pick_heading)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(hook_x_coordinate, hook_y_coordinate, Math.toRadians(hook_heading)))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> {RunHookPosition();})
                .lineToLinearHeading(new Pose2d(hook_x_coordinate, hook_y_coordinate+20, Math.toRadians(hook_heading)))
                .addTemporalMarker(() -> {Release();})
                //
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


    private void RunHookPosition() {
        Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, RobotActionConfig.deposit_Slide_UpLiftPower);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);// start after wait seconds
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
    }

    private void Release() {
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
        sleep((long) 1.8);
    }

    private void RunSpecimenPosition() {
        Slides_Move(RobotActionConfig.deposit_Slide_Down_Pos, RobotActionConfig.deposit_Slide_DownLiftPower);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Pick);// start after wait seconds
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Pick);
    }


}
