package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Auto.AprilTagAuto.AprilTagAuto;

@TeleOp (name= "AprilTag_OpMode", group = "org/firstinspires/ftc/teamcode/OpMode")
public class TestBotOpMode extends OpMode {

    public TestBotHardware robot;
    public RobotMovement robotMovement;
    public AprilTagAuto aprilTag;

    public GamepadEx gamepadCo1;                    //For gamepad
    public GamepadEx gamepadCo2;

    public void init () {
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        robot = new TestBotHardware();
        robot.init(hardwareMap);
        robot.initIMU();

        robotMovement = new RobotMovement(robot, gamepadCo1, gamepadCo2);

        aprilTag = new AprilTagAuto(hardwareMap);
        aprilTag.init();

        telemetry.addLine("RobotInitialized");
        telemetry.update();
    }

    public void loop() {
        robotMovement.robotDriveTrain();

        aprilTag.tagID();
        aprilTag.robotFieldCoordinate();

        telemetry.addData("AprilTag coordinates x axis", aprilTag.aprilTagUpdate()[0]);
        telemetry.addData("AprilTag coordinates y axis", aprilTag.aprilTagUpdate()[1]);
        telemetry.addData("AprilTag Info (Robot x axis)", aprilTag.tagAxis()[0]);
        telemetry.addData("AprilTag Info (Robot y axis)", aprilTag.tagAxis()[1]);
        telemetry.addData("AprilTag Info (Robot bearing)", aprilTag.tagAxis()[2]);
        telemetry.addData("AprilTag Info (Robot yaw)", aprilTag.tagAxis()[3]);
        telemetry.addData("TagID", aprilTag.tagID());
        telemetry.addLine("________________Field Coordinate__________________");
        telemetry.addData("Robot Field Coordinate x axis", aprilTag.robotFieldCoordinate().get("x"));
        telemetry.addData("Robot Field Coordinate y axis", aprilTag.robotFieldCoordinate().get("y"));
        telemetry.update();
    }
}
