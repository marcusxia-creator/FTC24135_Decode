package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp (name= "Robot_OpMode", group = "RobotOpMode")
public class RobotOpMode extends OpMode {

    //Declare all of the classes
    public RobotHardware robot;
    public RobotMovement robotMovement;
    public RobotIntake robotIntake;

    @Override
    public void init() {
        //Initializing the robot
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();

        //Initializing robot drive train
        robotMovement = new RobotMovement(gamepad1, gamepad2, robot);

        //Initializing robot intake
        robotIntake = new RobotIntake(gamepad1, gamepad2, robot);
        robotIntake.intakeInit(RobotActionConfig.intake_Arm_Retract);

        //Initializing FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("RobotInitialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Loop the robot functions
        robotMovement.robotDriveTrain();
        robotIntake.intakeSlideControl();
    }
}
