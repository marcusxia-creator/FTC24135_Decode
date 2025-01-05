package org.firstinspires.ftc.teamcode.TeleOps.ApirlTag;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    boolean drive = false;
    boolean goLeft = false;
    boolean goRight = false;
    boolean goForward = false;
    boolean goBackward = false;

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

                if (!drive) {
                    if (tag.ftcPose.x < 0.3) {
                        strafeLeft();
                    }
                    if (tag.ftcPose.x > 0.3) {
                        strafeRight();
                    }
                    else {
                        stopRobot();
                        drive = true;
                    }
                }
                if (drive) {
                    if (tag.ftcPose.y > 16) {
                        driveForward();
                    }
                    if (tag.ftcPose.y < 15) {
                        driveBackward();
                    }
                    else {
                        stopRobot();
                        drive = false;
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
