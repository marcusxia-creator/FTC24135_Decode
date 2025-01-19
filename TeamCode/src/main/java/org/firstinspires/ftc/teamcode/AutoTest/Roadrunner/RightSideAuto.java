
package org.firstinspires.ftc.teamcode.AutoTest.Roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="RightSideAuto", group="org.firstinspires.ftc.teamcode.AutoTest.Roadrunner")
@Config
public class RightSideAuto extends LinearOpMode {

    public static double highbar_x_coordinate = 0;
    public static double highbar_y_coordinate = -32;
    public static double highbar_prep_x_coordinate = 0;
    public static double highbar_prep_y_coordinate = -44;
    public static double specimen_pickup_x_coordinate = 36;
    public static double specimen_pickup_y_coordinate = -58;
    public static double first_move_x_coordinate = 7.5;
    public static double first_move_y_coordinate = -36;
    public static double second_move_x_coordinate = 36;
    public static double second_move_y_coordinate = -36;
    public static double third_move_x_coordinate = 36;
    public static double third_move_y_coordinate = -8;
    public static double fourth_move_x_coordinate = 48;
    public static double fourth_move_y_coordinate = -8;
    public static double fifth_move_x_coordinate = 48;
    public static double fifth_move_y_coordinate = -48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        RobotActionConfig value = new RobotActionConfig();
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(7.5, -64, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(first_move_x_coordinate, first_move_y_coordinate, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(second_move_x_coordinate, second_move_y_coordinate, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(third_move_x_coordinate, third_move_y_coordinate, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(fourth_move_x_coordinate, fourth_move_y_coordinate, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(fifth_move_x_coordinate, fifth_move_y_coordinate, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(highbar_prep_x_coordinate, highbar_prep_y_coordinate, Math.toRadians(-270)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-270)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate, specimen_pickup_y_coordinate, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(highbar_prep_x_coordinate, highbar_prep_y_coordinate, Math.toRadians(-270)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate, highbar_y_coordinate, Math.toRadians(-270)))
                .waitSeconds(1)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}