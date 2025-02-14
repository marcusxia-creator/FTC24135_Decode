package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    public AprilTag aprilTag;

    public GamepadEx gamepadCo1;                    //For gamepad
    public GamepadEx gamepadCo2;

    public RUNMODE runMode = RUNMODE.RUN;

    @Override
    public void init() {
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        //Initializing the robot
        robot = new RobotHardware();
        robot.init(hardwareMap);
        robot.initIMU();

        //Initializing robot drive train
        /** Change it back !!! robotMovement = new RobotMovement(robot, gamepadCo1, gamepadCo2); **/

        //Initializing robot intake
        robotIntake = new RobotIntake(robot, gamepadCo1, gamepadCo2, gamepad1, gamepad2);
        robotIntake.intakeInit();

        //Initializing deposit
        robotDeposit = new RobotDeposit(robot, gamepadCo1, gamepadCo2);
        robotDeposit.init();

        aprilTag = new AprilTag(hardwareMap);
        aprilTag.init();

        telemetry.addLine("RobotInitialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        //Loop the robot functions
        robotMovement.robotDriveTrain();


        // Toggle run mode
        if (gamepadCo1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.6 && gamepadCo2.getButton(GamepadKeys.Button.BACK)) {

            ToggleRunMode();
        }

        //Run base on run mode
        if (runMode == RUNMODE.RUN){
            robotIntake.intakeSlideControl();
            //robotDeposit.depositBarState();
            //robotDeposit.depositBasketState();
        }
        else {
            //Servotest.test();
        }

        aprilTag.aprilTagUpdate();
        aprilTag.tagAxis();

        telemetry.addData("Intake Slide State", robotIntake.intakeState);
        telemetry.addData("AprilTag coordinates", aprilTag.aprilTagUpdate());
        telemetry.addData("AprilTag Info", aprilTag.tagAxis());
    }

    public enum RUNMODE {
        TEST,
        RUN
    }

    private void ToggleRunMode() {
        if (runMode == RUNMODE.RUN) {
            runMode = RUNMODE.TEST;
        } else {
            runMode = RUNMODE.RUN;
        }
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
