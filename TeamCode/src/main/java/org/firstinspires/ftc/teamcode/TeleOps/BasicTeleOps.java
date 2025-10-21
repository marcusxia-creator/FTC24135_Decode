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

import java.util.List;

@Config
@TeleOp(name = "TeleOps_Decod_gw", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOps extends OpMode {

    public enum ControlState { RUN, TEST }

    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private ServoTest servoTest;

    private ControlState controlState = ControlState.RUN;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private boolean lBstartPressed = false;
    private List<LynxModule> allHubs;



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);


        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();



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


        telemetry.addLine("-------------------");
        telemetry.addData("Status", "initialized");
        telemetry.addData("Control Mode", robotDrive.getDriveMode().name());
        telemetry.addData("Shooter Power", robot.shooterMotor.getPower());
        telemetry.addData("Intake Motor Power", robot.intakeMotor.getPower());
        telemetry.update();
    }
    @Override
    public void start() {
       ;    // ← reset your timer right when start() is called
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



        /// Drive train control
        robotDrive.DriveLoop();

        if (gamepadCo1.getButton(BACK) && gamepadCo1.getButton(LEFT_BUMPER) && isButtonDebounced()) {
            /// LEFT BUMPER + BACK Button for lower deposit

        }

        /// BACK Button to lower the slides
        if (gamepadCo1.getButton(BACK) && !gamepadCo1.getButton(LEFT_BUMPER) && isButtonDebounced()) {
            debounceTimer.reset();

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


        }
        /// Control condition -  Check TEST Status for Servo Test
        if (controlState == ControlState.TEST) {
            servoTest.loop();
        }

        telemetry.addLine("--------------Op Mode--------------");
        telemetry.addData("Run Mode", controlState);
        telemetry.addData("Drive Mode", robotDrive.getDriveMode().name());


        telemetry.addData("Heading", robot.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addLine("--------Deposit-------------");
        telemetry.addData("Deposit Arm Position", robot.depositLeftArmServo.getPosition());
        telemetry.addData("Deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("Deposit Claw Position", robot.depositClawServo.getPosition());
        telemetry.addLine("--------Intake-------------");


        telemetry.update();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.shooterMotor.setPower(0);
        robot.intakeMotor.setPower(0);
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
