
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="RightSideAuto", group="org.firstinspires.ftc.teamcode.Auto")
@Config
public class RightSideAuto extends LinearOpMode {

    public static double highbar_x_coordinate = -3;
    public static double highbar_y_coordinate = -33;
    public static double specimen_pickup_x_coordinate = 27;
    public static double specimen_pickup_y_coordinate = -51;
    public static double first_sample_pickup_x_coordinate = 26;
    public static double first_sample_pickup_y_coordinate = -38;

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
        robot.intakeWristServo.setPosition(0.3);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
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
                            //.waitSeconds(vSlideWaitTimer)
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
                .waitSeconds(waitTimer)
                //back out of bar position -1st time
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, -60, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .waitSeconds(waitTimer)
                //drop slides and put arm back to transfer position
                .addTemporalMarker(() -> {
                    Slides_Move(-950, 0.8);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .waitSeconds(vSlideWaitTimer)
                .addTemporalMarker(() -> {
                    Slides_Stop();
                })
                .waitSeconds(waitTimer)
                /** 2nd Segment --> push red sample for specimen
                 *
                 */
                //.lineToLinearHeading(new Pose2d(36, -48, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(55.5, -8, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(55.5, -56, Math.toRadians(-90)))
                //.lineToLinearHeading(new Pose2d(49, -56, Math.toRadians(-90)))

                /** 3nd Segment --> move to pick 1st Specimen*/
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //Grab 1st specimen
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                })
                .waitSeconds(1.0+hSlideWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(waitTimer)
                /** 3.1 segment ---->  Transfer 2nd specimen*/
                .addTemporalMarker(() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
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
                .UNSTABLE_addTemporalMarkerOffset(hSlideWaitTimer+clawOpenTimer, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(1)
                //extend slides to scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(vSlideWaitTimer)
                //move to highbar spot
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
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
                .waitSeconds(waitTimer*2)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate - 10, Math.toRadians(-90)))

                /** 4rd Segment --> go back to pickup position - */
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                //drop slides and go back to transfer position
                .addTemporalMarker(() -> {
                    Slides_Move(-950, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .waitSeconds(vSlideWaitTimer)
                .addTemporalMarker(() -> {
                    Slides_Stop();
                })
                /** pick up 3rd specimen*/
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                })
                .waitSeconds(1.0+hSlideWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(waitTimer-0.05)

                /** 3.1 segment ---->  Transfer 1st specimen*/
                .addTemporalMarker(() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
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
                .UNSTABLE_addTemporalMarkerOffset(hSlideWaitTimer+clawOpenTimer, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(1)
                //extend slides to scoring position
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(vSlideWaitTimer)
                //move to highbar spot
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
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
                .waitSeconds(waitTimer*2)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate - 10, Math.toRadians(-90)))

                /** move to OB ZONE*/
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))

                /** ----> Extend slides to OB ZONE */
                .addTemporalMarker(() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .waitSeconds(0.7)
                /**. can add another pick up and score movement.
                 *
                 */

                /**
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.5)
                //Transfer second specimen
                .addTemporalMarker(() -> {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })

                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(0.9)
                .addTemporalMarker(() -> {
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .addTemporalMarker(() -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setDrivePower(new Pose2d(0, 0, 0));
                })
                .addTemporalMarker(() -> {
                    Slides_Move(-1000, 0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    Slides_Stop();
                })
                .addTemporalMarker(() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .waitSeconds(0.7)
                 */
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);

        while (opModeIsActive()) {
            telemetry.addData("Slides Position: ", robot.liftMotorLeft.getCurrentPosition());
        }
    }

    private void Slides_Move(int dist, double speed) {
        robot.liftMotorLeft.setTargetPosition(dist);
        robot.liftMotorRight.setTargetPosition(dist);
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