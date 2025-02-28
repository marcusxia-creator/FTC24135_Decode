package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;

import java.util.Locale;

@TeleOp(name = "Test_Pinpoint_Odometry", group = "TeleOps")
public class PinpointLocalizerTest extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        //✅ Initialize PinPoint Odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"Pinpoint");
         /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-149.225, -165.1); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        //odo.setEncoderResolution(13.26291192);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
        final Pose2D iniPose =new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,0);
        odo.setPosition(iniPose);

        // ✅ Initialize SampleMecanumDrive
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();

        if (isStopRequested()) return;

        // ✅ Set an initial pose (if needed)
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        while (opModeIsActive()) {
            // ✅ Update drive and odometry
            odo.update();
            drive.update();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.right_stick_y*0.5,
                            -gamepad1.right_stick_x*0.5,
                            -gamepad1.left_stick_x*0.5
                    )
            );

            if (gamepad1.a){
                drive.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
                odo.resetPosAndIMU();
            }

            // ✅ Retrieve the current position
            Pose2d pose = drive.getPoseEstimate();
            Pose2d velocity = drive.getPoseVelocity();

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            // ✅ Display the position and velocity in telemetry
            telemetry.addLine("---------Roadrunner---------");
            telemetry.addData("X inch", pose.getX());
            telemetry.addData("Y inch", pose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Velocity X inch", velocity.getX());
            telemetry.addData("Velocity Y inch", velocity.getY());
            telemetry.addData("Velocity Heading (Radians/sec)", velocity.getHeading());
            telemetry.addData("Velocity Heading (deg/sec)", Math.toDegrees(velocity.getHeading()));
            telemetry.addLine("---------PinPoint X, Y---------");
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X inch: %.3f, Y inch: %.3f, H Radian %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS));
            String data2 = String.format(Locale.US, "{X mm: %.3f, Y mm: %.3f, H degree: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position in inch", data);
            telemetry.addData("Position in mm", data2);
            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            telemetry.addLine("---------PinPoint Velocity X, Y ---------");
            Pose2D vel = odo.getVelocity();
            String odoVel = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.RADIANS));
            String odoVel2 = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity in inch", odoVel);
            telemetry.addData("Velocity in mm", odoVel2);

            /*
            Get Raw Tick Readings
             */
            telemetry.addLine("---------Encoder X, Y---------");
            telemetry.addData("Encoder X tick", odo.getEncoderX());
            telemetry.addData("Encoder Y tick", odo.getEncoderX());

            telemetry.addLine("---------Frequency--------");
            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();
        }
    }
}

