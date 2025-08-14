package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.drive.StandardTrackingWheelLocalizer;

import java.util.List;

@Config
@TeleOp(name = "TeleOps_Premier_gw", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOps extends OpMode {

    public enum ControlState { RUN, TEST }

    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private FiniteStateMachineDeposit depositArmDrive;
    private FiniteStateMachineIntake intakeArmDrive;
    private ServoTest servoTest;
    private SlidesPIDControl slidePIDControl;
    private ControlState controlState = ControlState.RUN;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private boolean lBstartPressed = false;
    private List<LynxModule> allHubs;

    public static double kp = 3.0;
    public static double ki = 0.0;
    public static double kd = 0.05;
    public static double fu =0.22;
    public static double fd =-0.1;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        slidePIDControl = new SlidesPIDControl(robot,kp,ki,kd,fu,fd,RobotActionConfig.TICKS_PER_MM_SLIDES*RobotActionConfig.deposit_Slide_Highbasket_Pos,RobotActionConfig.TICKS_PER_MM_SLIDES);

        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        depositArmDrive = new FiniteStateMachineDeposit(robot, gamepadCo1, gamepadCo2, intakeArmDrive, telemetry, slidePIDControl);
        ///depositArmDrive.ArmInit(); did not initiate depositArm at the beginning of TeleOps

        intakeArmDrive = new FiniteStateMachineIntake(robot, gamepadCo1, gamepadCo2, depositArmDrive);
        intakeArmDrive.Init();

        servoTest = new ServoTest(robot, gamepadCo1, gamepadCo2);

        /// Get all hubs from the hardwareMap
        allHubs = hardwareMap.getAll(LynxModule.class);
        ///Set bulk caching mode to Auto
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ///
        int lfPos = robot.frontLeftMotor.getCurrentPosition();
        int rfPos = robot.frontRightMotor.getCurrentPosition();
        int lbPos = robot.backLeftMotor.getCurrentPosition();
        int rbPos = robot.backRightMotor.getCurrentPosition();

        /// Lift motor encouder reading
        int liftmotor_left_pos = robot.liftMotorLeft.getCurrentPosition();
        int liftmotor_right_pos = robot.liftMotorRight.getCurrentPosition();
        telemetry.addLine("-------------------");
        telemetry.addData("Status", "initialized");
        telemetry.addData("Control Mode", robotDrive.getDriveMode().name());
        telemetry.addData("VS Left Encoder", liftmotor_left_pos);
        telemetry.addData("VS Right Encoder", liftmotor_right_pos);
        telemetry.update();
    }
    @Override
    public void start() {
        depositArmDrive.resetHangTimer();    // ← reset your timer right when start() is called
    }

    @Override
    public void loop() {
        /**
        /// Lynx for Motor encoder reading
        for (LynxModule hub : allHubs) {
            BulkData bulkData = hub.getBulkData();
            if (bulkData != null) {
                if (hub.equals(allHubs.get(0))) {
                    telemetry.addData("Drive FL Pos", bulkData.getMotorCurrentPosition(robot.frontLeftMotor.getPortNumber()));
                    telemetry.addData("Drive FR Pos", bulkData.getMotorCurrentPosition(robot.frontRightMotor.getPortNumber()));
                    telemetry.addData("Drive BL Pos", bulkData.getMotorCurrentPosition(robot.backLeftMotor.getPortNumber()));
                    telemetry.addData("Drive BR Pos", bulkData.getMotorCurrentPosition(robot.backRightMotor.getPortNumber()));
                } else {
                    telemetry.addData("Lift L Pos", bulkData.getMotorCurrentPosition(robot.liftMotorLeft.getPortNumber()));
                    telemetry.addData("Lift R Pos", bulkData.getMotorCurrentPosition(robot.liftMotorRight.getPortNumber()));
                }
            }
        }
        */

        int lfPos = robot.frontLeftMotor.getCurrentPosition();
        int rfPos = robot.frontRightMotor.getCurrentPosition();
        int lbPos = robot.backLeftMotor.getCurrentPosition();
        int rbPos = robot.backRightMotor.getCurrentPosition();

        /// Lift motor encouder reading
        int liftmotor_left_pos = robot.liftMotorLeft.getCurrentPosition();
        int liftmotor_right_pos = robot.liftMotorRight.getCurrentPosition();
        double vs_L_mm = liftmotor_left_pos/RobotActionConfig.TICKS_PER_MM_SLIDES;
        double vs_R_mm = liftmotor_right_pos/RobotActionConfig.TICKS_PER_MM_SLIDES;

        /// Drive train control
        robotDrive.DriveLoop();

        if (gamepadCo1.getButton(BACK) && gamepadCo1.getButton(LEFT_BUMPER) && isButtonDebounced()) {
            resetLiftEncoders();
            /// LEFT BUMPER + BACK Button for lower deposit
            slidePIDControl.setEnabled(false);
            depositArmDrive.ArmInit();
        }

        /// BACK Button to lower the slides
        if (gamepadCo1.getButton(BACK) && !gamepadCo1.getButton(LEFT_BUMPER) && isButtonDebounced()) {
            debounceTimer.reset();
            slidePIDControl.setEnabled(false);
            lowerDepositSlide();
        }

        /// START BUTTON + LEFT BUMPER to toggle control state
        if (gamepadCo1.getButton(START) && gamepadCo1.getButton(LEFT_BUMPER) && !lBstartPressed) {
            toggleControlState();
            debounceTimer.reset();
            lBstartPressed = true;
        } else if (!gamepadCo1.getButton(START) || !gamepadCo1.getButton(LEFT_BUMPER)) {
            lBstartPressed = false;
        }

        /// Control condition -  Check Run Status
        if (controlState == ControlState.RUN) {
            depositArmDrive.DepositArmLoop();
            intakeArmDrive.IntakeArmLoop();

            telemetry.addData("Deposit State", depositArmDrive.liftState);
            telemetry.addData("Deposit Claw State", depositArmDrive.depositClawState);
            telemetry.addData("Intake State", intakeArmDrive.intakeState);
            telemetry.addData("Intake Claw State", intakeArmDrive.intakeClawState);
        }
        /// Control condition -  Check TEST Status for Servo Test
        if (controlState == ControlState.TEST) {
            servoTest.loop();
        }

        telemetry.addLine("--------------Op Mode--------------");
        telemetry.addData("Run Mode", controlState);
        telemetry.addData("Drive Mode", robotDrive.getDriveMode().name());
        telemetry.addData("VS Left mm", vs_L_mm);
        telemetry.addData("VS Right mm", vs_R_mm);
        telemetry.addData("VS Left tick", liftmotor_left_pos);
        telemetry.addData("VS Right tick", liftmotor_right_pos);
        telemetry.addData("targetTicks",slidePIDControl.getTargetTicks());
        telemetry.addData("At targetTicks",slidePIDControl.atTarget());
        telemetry.addData("slides power",slidePIDControl.getpower());
        telemetry.addData("slides ff value",slidePIDControl.getf());

        telemetry.addData("Heading", robot.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addLine("--------Deposit-------------");
        telemetry.addData("Deposit Arm Position", robot.depositLeftArmServo.getPosition());
        telemetry.addData("Deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("Deposit Claw Position", robot.depositClawServo.getPosition());
        telemetry.addLine("--------Intake-------------");
        telemetry.addData("Intake Arm Left Position", robot.intakeArmServo.getPosition());
        telemetry.addData("Intake Wrist Position", robot.intakeWristServo.getPosition());
        telemetry.addData("Intake Claw Position", robot.intakeClawServo.getPosition());
        telemetry.addData("Intake Slide Left Position", robot.intakeLeftSlideServo.getPosition());
        telemetry.addData("Intake Slide Right Position", robot.intakeRightSlideServo.getPosition());
        telemetry.addData("Intake Turret Position", robot.intakeTurretServo.getPosition());
        telemetry.addData("Intake Rotation Position", robot.intakeRotationServo.getPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();
    }
    /// Helper -- toggle control state - RUN vs TEST
    private void toggleControlState() {
        controlState = (controlState == ControlState.RUN) ? ControlState.TEST : ControlState.RUN;
    }

    /**
    private boolean LSisPressed() {
        return robot.limitSwitch.getState();
    }
     */

    /// Helper -- lower deposit slide to bottom
    private void lowerDepositSlide() {
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorLeft.setPower(-0.3);
        robot.liftMotorRight.setPower(-0.3);
    }

    /// Helper -- to reset deposit slide motor encoder
    private void resetLiftEncoders() {
        robot.liftMotorLeft.setPower(0.0);
        robot.liftMotorRight.setPower(0.0);
        // 3) Now that we're at zero, actually reset the encoder count
        robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // 4) Return to a normal run mode if you still want to drive by power/velocity
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /// Helper -- to Button Debouncer
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    /// Add a voltage sensor
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0 && voltage < result) {
                result = voltage;
            }
        }
        // If we never found a positive reading, default to 0
        return (result == Double.POSITIVE_INFINITY) ? 0.0 : result;
    }

}
