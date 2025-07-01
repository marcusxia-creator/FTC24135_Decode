package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class ServoTest {

    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private RobotHardware robot;

    private double servoposition;
    private double servoposition2;
    private static final int delta_Position = 50;
    private int current_Position;

    private static final double speed = 1.0;

    private final ElapsedTime debounceTimer = new ElapsedTime();
    private static final double DEBOUNCE_THRESHOLD = 0.25;

    private static boolean leftButtonPressed = false;

    public ServoTest(RobotHardware robot, GamepadEx gamepad1, GamepadEx gamepad2) {
        this.gamepad_1 = gamepad1;
        this.gamepad_2 = gamepad2;
        this.robot = robot;
    }

    public void init() {

        servoposition = 0.0;
        servoposition2 = 0.0;

        // Initialize servos
        robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);

        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
        robot.intakeLeftSlideServo.setPosition(0.0);
        robot.intakeRightSlideServo.setPosition(0.0);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.intakeTurretServo.setPosition(RobotActionConfig.intake_Turret_Mid);

        // Initialize lift motors
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotorLeft.setTargetPosition(20);
        robot.liftMotorRight.setTargetPosition(20);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
    }


    public void loop() {
        // --- Deposit System ---
        if (gamepad_1.getButton(A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        }

        if (gamepad_1.getButton(B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        }

        if (gamepad_1.getButton(DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositLeftArmServo.getPosition() + 0.01;
            robot.depositLeftArmServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.depositRightArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositLeftArmServo.getPosition() - 0.01;
            robot.depositLeftArmServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.depositRightArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition() + 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition, 0, 1));
            leftButtonPressed = true;
        }

        if (gamepad_1.getButton(DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition() - 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            current_Position = robot.liftMotorLeft.getCurrentPosition();


            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position + delta_Position,20,10000));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position + delta_Position,20,10000));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
           // robot.liftMotorRight.setPower(speed);
        }

        if (gamepad_1.getButton(X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            current_Position = robot.liftMotorLeft.getCurrentPosition();


            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position - delta_Position,20,10000));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position - delta_Position,20,10000));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            // robot.liftMotorRight.setPower(speed);
        }

        // --- Intake System ---
        if (gamepad_2.getButton(DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeArmServo.getPosition() + 0.01;
            robot.intakeArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeArmServo.getPosition() - 0.01;
            robot.intakeArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        }

        if (gamepad_2.getButton(B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
        }

        if (gamepad_2.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition() - 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition() + 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeRotationServo.getPosition() + 0.01;
            robot.intakeRotationServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeRotationServo.getPosition() - 0.01;
            robot.intakeRotationServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_2.getButton(LEFT_BUMPER) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeLeftSlideServo.getPosition() - 0.01;
            servoposition = robot.intakeRightSlideServo.getPosition() - 0.01;
            robot.intakeLeftSlideServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.intakeRightSlideServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_2.getButton(RIGHT_BUMPER) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeLeftSlideServo.getPosition() + 0.01;
            servoposition = robot.intakeRightSlideServo.getPosition() + 0.01;
            robot.intakeLeftSlideServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.intakeRightSlideServo.setPosition(Range.clip(servoposition, 0, 1));

        }
    }
}
