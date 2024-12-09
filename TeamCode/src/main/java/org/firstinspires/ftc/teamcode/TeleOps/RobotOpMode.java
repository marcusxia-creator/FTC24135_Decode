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

    //Declare all of the variables
    private static final double motorMaxSpeed = 0.4;

    private static final double intakeSlideExtendedPosition = 0.55;
    private static final double intakeSlideRetractedPosition = 0.3;

    private static final double intakeArmExtendedPosition = 0.05;
    private static final double intakeArmRetractedPosition = 0.53;
    private static final double intakeArmIdlePosition = 0.1;

    private static final double intakeRotationServoNeutralPosition = 0.49;

    private static final double intakeClawServoClosePosition = 0.3;
    private static final double intakeClawServoOpenPosition = 0.55;

    private static final double intakeArmServoChangeAmount = 0.05;
    private static final double intakeRotationServoSteerAmount = 0.1;

    private static final double depositServoClawClosePosition = 0.0;

    private static final double debounce_threshold = 0.25;

    @Override
    public void init() {
        //Initializing the robot
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();

        //Initializing robot drive train
        robotMovement = new RobotMovement(gamepad1, gamepad2, robot, motorMaxSpeed);

        //Initializing robot intake
        robotIntake = new RobotIntake
            (
                gamepad1, gamepad2, robot,
                intakeSlideExtendedPosition, intakeSlideRetractedPosition,
                intakeArmExtendedPosition, intakeArmRetractedPosition,
                intakeRotationServoNeutralPosition,
                intakeClawServoClosePosition, intakeClawServoOpenPosition,
                intakeArmServoChangeAmount, intakeRotationServoSteerAmount,

                intakeArmIdlePosition,

                depositServoClawClosePosition,

                debounce_threshold
            );

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
