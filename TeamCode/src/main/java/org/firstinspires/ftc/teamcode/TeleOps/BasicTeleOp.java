package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler1;

@TeleOp (name = "Basic TeleOp", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;

    private FSMIntake intakeControl;
    private FSMShooterManual shooterControl;
    private FSMAprilTagProc aprilTagProc;
    private IceWaddler1 iceWaddler;

    private FtcDashboard dashboard;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        aprilTagProc = new FSMAprilTagProc(robot);
        aprilTagProc.init();

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init(aprilTagProc);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        aprilTagProc.loop();
        robotDrive.DriveLoop();

        //Telemetry
        telemetry.addData("State","Running");
        if(robotDrive.autoHeading) {
            telemetry.addData("Heading Control", "Auto");
        }
        else{
            telemetry.addData("Heading Control", "Manual");
        }

        if(aprilTagProc.Detected){
            telemetry.addData("AprilTag", "Detected");
            telemetry.addData("AprilTag ID", aprilTagProc.tag.id);
            telemetry.addData("AprilTag Heading", aprilTagProc.Heading);
            telemetry.addData("AprilTag Distance", aprilTagProc.Distance);
            telemetry.addData("AprilTag Heading Correction", aprilTagProc.Heading*RobotActionConfig.autoHeadingCoeff);
        }
        else{
            telemetry.addData("AprilTag", "No Detection");
        }
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }
}
