package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp (name = "TestTeleOp", group = "org.firstinspires.ftc.teamcode")
public class TestTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private double servoposition;
    private double speed;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private RobotDrive robotDrive;

     // Example: gobilda 1150= 145.1 ticks/rev, 6000=28, GoBilda 5202/5203 = 537.7 ticks/rev.for 312rpm
    public static double SHOOTER_TICKS_PER_REV = 28;   // change to your motor
    public static double INTAKE_TICKS_PER_REV = 145.1;   // change to your motor
    public static double SHOOTER_RPM_CONVERSION = 60.0 / SHOOTER_TICKS_PER_REV;
    public static double INTAKE_RPM_CONVERSION = 60.0 / INTAKE_TICKS_PER_REV;

    private double shooter_rpm;
    private double intake_rpm;

    //
    private ElapsedTime jamTimer = new ElapsedTime();

    // Determine if Intake is jammed


    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servoposition = 0.0;
        speed = 0.0;

        robot.pushRampServo.setPosition(RobotActionConfig.rampResetPos);

        robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
        robot.rightGateServo.setPosition(RobotActionConfig.gateDown);
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);

        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {


        /// === Read velocity in ticks/sec and convert to RPM ===


        ///set a jammed boolean for determination


        ///
        if (gamepad_1.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            robot.pushRampServo.setPosition(RobotActionConfig.rampUpPos);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
            robot.pushRampServo.setPosition(RobotActionConfig.rampResetPos);
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() + 0.5;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() - 0.5;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && isButtonDebounced()) {
            servoposition = robot.leftGateServo.getPosition() + 0.01;
            robot.leftGateServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.rightGateServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && isButtonDebounced()) {
            servoposition = robot.leftGateServo.getPosition() - 0.01;
            robot.leftGateServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.rightGateServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_2.getButton(GamepadKeys.Button.X) && isButtonDebounced()){
            speed = robot.shooterMotor.getPower() + 0.05;
            robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.shooterMotor.setPower(Range.clip(speed,0.5,1.0));
        }

        if (gamepad_2.getButton(GamepadKeys.Button.Y) && isButtonDebounced()){
            robot.shooterMotor.setPower(0);
        }

        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()){
            speed = robot.intakeMotor.getPower() + 0.05;
            robot.intakeMotor.setPower(Range.clip(speed,0.5,1.0));
        }

        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()){
            robot.intakeMotor.setPower(0);
        }

        /**
        if (jammed) {


        }
         */

        // Telemetry (DS + Dashboard)       
        telemetry.addData("Ramp Position", robot.pushRampServo.getPosition());
        telemetry.addData("Left Gate Position", robot.leftGateServo.getPosition());
        telemetry.addData("Right Gate Position", robot.rightGateServo.getPosition());
        telemetry.addData("Spindexer Position", robot.spindexerServo.getPosition());
        telemetry.addLine("----Shooter----");
        telemetry.addData("Shooter Speed", robot.shooterMotor.getPower());
        telemetry.addData("Shooter Ticks/sec", "%.1f", shooter_ticksPerSec);
        telemetry.addData("Shooter RPM", "%.1f", shooter_rpm);
        telemetry.addLine("----Intake----");
        telemetry.addData("Intake Jam Status", jammed);
        telemetry.addData("Intake Speed", robot.intakeMotor.getPower());
        telemetry.addData("Intake Ticks/sec", "%.1f", intake_ticksPerSec);
        telemetry.addData("Intake RPM", "%.1f", intake_rpm);

        telemetry.update();
    }

    // Button Debounce Helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    // Intake Jam Helper
    private boolean isJammed() {
        if (  ) {
            if (jamTimer.seconds() > 0.3) return true; // jam confirmed for 0.3s
        } else {
            jamTimer.reset();
        }
        return false;
    }
    
}


