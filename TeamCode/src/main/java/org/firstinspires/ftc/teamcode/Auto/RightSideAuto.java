
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="RightSideAuto", group="org.firstinspires.ftc.teamcode.AutoTest.Roadrunner")
@Config
public class RightSideAuto extends LinearOpMode {

    public static double highbar_x_coordinate = -3;
    public static double highbar_y_coordinate = -33;
    public static double specimen_pickup_x_coordinate = 24;
    public static double specimen_pickup_y_coordinate = -48;
    public static double first_sample_pickup_x_coordinate = 34;
    public static double first_sample_pickup_y_coordinate = -38;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        RobotActionConfig value = new RobotActionConfig();
        RobotHardware robot = new RobotHardware();
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
                .addTemporalMarker(()->{
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos,0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(1.3)
                //drive to bar
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate,highbar_y_coordinate,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                //open claw
                .addTemporalMarker(()->{
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(1)
                //back out of bar position
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate - 10, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                .waitSeconds(1)
                //drop slides and put arm back to transfer position
                .addTemporalMarker(()->{
                    Slides_Move(-950,0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .waitSeconds(2)
                //move to first sample pickup
                .lineToLinearHeading(new Pose2d(first_sample_pickup_x_coordinate,first_sample_pickup_y_coordinate,Math.toRadians(40)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                //extend slides
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid+0.15);
                })
                .waitSeconds(0.7)
                //grab first sample
                .addTemporalMarker(()->{
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.3)
                //turn and drop into human player zone
                .turn(Math.toRadians(-90))
                .addTemporalMarker(()->{
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(0.3)
                //retrat intake slides
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                //move to pickup position
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                //Grab first specimen
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.5)
                //Transfer first specimen
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})
                .waitSeconds(0.9)
                //extend slides to scoring position
                .addTemporalMarker(()->{
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos,0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .waitSeconds(2)
                //move to highbar spot
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate,highbar_y_coordinate,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                //open claw and flat wrist
                .addTemporalMarker(()->{
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(1)
                //go back to pickup position
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                //drop slides and go back to transfer position
                .addTemporalMarker(()->{
                    Slides_Move(-950,0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .waitSeconds(2)
                //Extend slides to grab second specimen
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.5)
                //Transfer second specimen
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})
                .waitSeconds(0.9)
                .addTemporalMarker(()->{
                    Slides_Move(RobotActionConfig.deposit_Slide_Highbar_Pos,0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate,highbar_y_coordinate,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                .addTemporalMarker(()->{
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                .addTemporalMarker(()->{
                    Slides_Move(-1000,0.9);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                })
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .waitSeconds(0.7)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);

        while (opModeIsActive()){
            telemetry.addData("Slides Position: ",robot.liftMotorLeft.getCurrentPosition());
        };
    }

    private void Slides_Move(int dist, double speed) {
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        robot.liftMotorLeft.setTargetPosition(dist);
        robot.liftMotorRight.setTargetPosition(dist);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
        while (opModeIsActive() && (robot.liftMotorLeft.isBusy() && robot.liftMotorRight.isBusy())) {
            telemetry.addData("Slide mode","Running");
            telemetry.update();
        }
    }
}