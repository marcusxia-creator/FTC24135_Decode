package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp (name= "Robot_OpMode", group = "org/firstinspires/ftc/teamcode/OpMode")
public class RobotOpMode extends OpMode {

    //Declare all of the classes
    public RobotHardware robot;
    public RobotMovement robotMovement;
    public RobotIntake robotIntake;
    public RobotDeposit robotDeposit;

    public GamepadEx gamepadCo1;                    //For gamepad
    public GamepadEx gamepadCo2;

    @Override
    public void init() {
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        //Initializing the robot
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();

        //Initializing robot drive train
        robotMovement = new RobotMovement(robot, gamepadCo1, gamepadCo2);

        //Initializing robot intake
        robotIntake = new RobotIntake(robot, gamepadCo1, gamepadCo2);
        robotIntake.intakeInit();

        //Initializing deposit
        robotDeposit = new RobotDeposit(robot, gamepadCo1, gamepadCo2);
        robotDeposit.init();

        telemetry.addLine("RobotInitialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Loop the robot functions
        robotMovement.robotDriveTrain();
        robotIntake.intakeSlideControl();
        robotDeposit.depositBarState();
        robotDeposit.depositBasketState();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
    }
}
