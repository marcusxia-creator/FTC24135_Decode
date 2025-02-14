package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name= "AprilTag_OpMode", group = "org/firstinspires/ftc/teamcode/OpMode")
public class TestBotOpMode extends OpMode {

    public TestBotHardware robot;
    public RobotMovement robotMovement;
    public AprilTag aprilTag;

    public GamepadEx gamepadCo1;                    //For gamepad
    public GamepadEx gamepadCo2;

    public void init () {
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        robot = new TestBotHardware();
        robot.init(hardwareMap);
        robot.initIMU();

        robotMovement = new RobotMovement(robot, gamepadCo1, gamepadCo2);

        aprilTag = new AprilTag(hardwareMap);
        aprilTag.init();

        telemetry.addLine("RobotInitialized");
        telemetry.update();
    }

    public void loop() {
        robotMovement.robotDriveTrain();

        aprilTag.aprilTagUpdate();
        aprilTag.tagAxis();

        telemetry.addData("AprilTag coordinates", aprilTag.aprilTagUpdate());
        telemetry.addData("AprilTag Info", aprilTag.tagAxis()[0]);
        telemetry.addData("TagID", aprilTag.tagID());
        telemetry.update();
    }
}
