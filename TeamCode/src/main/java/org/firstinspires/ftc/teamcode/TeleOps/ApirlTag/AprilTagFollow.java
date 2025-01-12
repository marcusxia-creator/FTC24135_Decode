package org.firstinspires.ftc.teamcode.TeleOps.ApirlTag;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class AprilTagFollow extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;


    private static final ElapsedTime checkTime = new ElapsedTime();

    private static double FIRST_TIMER_THRESHOLD = 0;
    private static double TIMER_THRESHOLD = 5;

    private static boolean driveFront = true;
    private static boolean strafe = false;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Web_Cam"))
                .setCameraResolution(new Size(640, 480))
                .build();


        waitForStart();

        while (opModeIsActive()) {
            if (!tagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                if (tag.ftcPose.x < 0.3 && tag.ftcPose.x > -0.3 && tag.ftcPose.y < 16 && tag.ftcPose.y > 15) {
                    stopRobot();
                }
                else {
                    if (tag.ftcPose.y > 16 && driveFront) {
                        driveForward();
                        FIRST_TIMER_THRESHOLD = 5;
                    }
                    if (tag.ftcPose.y < 15 && checkTime.seconds() > FIRST_TIMER_THRESHOLD) {
                        driveBackward();
                        driveFront = false;
                    }
                    else {
                        strafe = true;
                        checkTime.reset();
                        FIRST_TIMER_THRESHOLD = 0;
                    }

                    if (tag.ftcPose.x < 0.3 && strafe && checkTime.seconds() > TIMER_THRESHOLD) {
                        strafeRight();
                        TIMER_THRESHOLD = 10;
                    }
                    if (tag.ftcPose.x > -0.3 && checkTime.seconds() > TIMER_THRESHOLD) {
                        strafeLeft();
                        strafe = false;
                    }
                    else {
                        driveFront = true;
                        checkTime.reset();
                        TIMER_THRESHOLD = 5;
                    }
                }

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
            }


            telemetry.addData("April tag is detected", !tagProcessor.getDetections().isEmpty());
            telemetry.update();
        }
    }

    private void strafeLeft () {
        frontLeftMotor.setPower(-0.1);
        backLeftMotor.setPower(0.1);
        frontRightMotor.setPower(0.1);
        backRightMotor.setPower(-0.1);
    }

    private void strafeRight () {
        frontLeftMotor.setPower(0.1);
        backLeftMotor.setPower(-0.1);
        frontRightMotor.setPower(-0.1);
        backRightMotor.setPower(0.1);
    }

    private void driveForward () {
        frontLeftMotor.setPower(0.1);
        backLeftMotor.setPower(0.1);
        frontRightMotor.setPower(0.1);
        backRightMotor.setPower(0.1);
    }

    private void driveBackward () {
        frontLeftMotor.setPower(-0.1);
        backLeftMotor.setPower(-0.1);
        frontRightMotor.setPower(-0.1);
        backRightMotor.setPower(-0.1);
    }

    private void stopRobot () {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
