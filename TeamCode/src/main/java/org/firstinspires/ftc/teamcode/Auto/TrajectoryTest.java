package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Auto.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class TrajectoryTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeToLinearHeading(new Vector2d(36,32),Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(36,48),Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(12,12),Math.toRadians(22.5))
                            .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(-45))
                            .strafeToLinearHeading(new Vector2d(12,32),Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(12,48),Math.toRadians(90))
                            .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(-45))
                            .strafeToLinearHeading(new Vector2d(10,12),Math.toRadians(-90))
                            .build());
        }
    }
}
