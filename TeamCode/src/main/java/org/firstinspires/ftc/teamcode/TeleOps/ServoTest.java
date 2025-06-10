package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@TeleOp(name = "Servo Test", group = "org.firstinspires.ftc.teamcode")
public class ServoTest extends OpMode {

    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private RobotHardware robot;

    private double servoposition;
    private double servoposition2;
    private static final int delta_Position = 50;
    private int current_Position;

    private static final double speed = 0.4;
    private static final double DEBOUNCE_THRESHOLD = 0.25;
    private final ElapsedTime debounceTimer = new ElapsedTime();

    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        robot = new RobotHardware(hardwareMap); // ✅ No longer shadowed
        robot.init(hardwareMap);

        servoposition = 0.0;
        servoposition2 = 0.0;

        // Initialize servos
        robot.depositLeftArmServo.setPosition(0.0);
        robot.depositRightArmServo.setPosition(0.0);
        robot.depositWristServo.setPosition(0.0);
        robot.depositClawServo.setPosition(0.0);

        robot.intakeArmServo.setPosition(0.0);
        robot.intakeLeftSlideServo.setPosition(0.0);
        robot.intakeRightSlideServo.setPosition(0.0);
        robot.intakeWristServo.setPosition(0.0);
        robot.intakeRotationServo.setPosition(0.0);
        robot.intakeClawServo.setPosition(0.0);
        robot.intakeTurretServo.setPosition(0.0);

        // Initialize lift motors
        robot.liftMotorLeft.setTargetPosition(50);
        robot.liftMotorRight.setTargetPosition(50);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);

    }

    @Override
    public void loop() {
        // --- Deposit System ---
        if (gamepad_1.getButton(A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositClawServo.getPosition() + 0.01;
            robot.depositClawServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositClawServo.getPosition() - 0.01;
            robot.depositClawServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositLeftArmServo.getPosition() + 0.01;
            robot.depositLeftArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositRightArmServo.getPosition() - 0.01;
            robot.depositRightArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition() + 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition() - 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            current_Position = robot.liftMotorLeft.getCurrentPosition();

            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position + delta_Position,50,3500));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position + delta_Position,50,3500));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);

        }

        if (gamepad_1.getButton(Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            current_Position = robot.liftMotorLeft.getCurrentPosition();

            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position - delta_Position,50,3500));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position - delta_Position,50,3500));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
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
            servoposition = robot.intakeClawServo.getPosition() + 0.01;
            robot.intakeClawServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeClawServo.getPosition() - 0.01;
            robot.intakeClawServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition() + 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition() - 0.01;
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
        if (gamepad_2.getButton(DPAD_LEFT) && gamepad_2.getButton(LEFT_BUMPER) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeTurretServo.getPosition() - 0.01;
            robot.intakeTurretServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_2.getButton(DPAD_RIGHT) && gamepad_2.getButton(LEFT_BUMPER) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeTurretServo.getPosition() + 0.01;
            robot.intakeTurretServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        // --- Telemetry ---
        telemetry.addData("VS Left Position", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("VS Right Position", robot.liftMotorRight.getCurrentPosition());
        telemetry.addLine("---------------------");
        telemetry.addData("Deposit Arm Position", robot.depositLeftArmServo.getPosition());
        telemetry.addData("Deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("Deposit Claw Position", robot.depositClawServo.getPosition());
        telemetry.addLine("---------------------");
        telemetry.addData("Intake Arm Left Position", robot.intakeArmServo.getPosition());
        telemetry.addData("Intake Wrist Position", robot.intakeWristServo.getPosition());
        telemetry.addData("Intake Claw Position", robot.intakeClawServo.getPosition());
        telemetry.addData("Intake Slide Left Position", robot.intakeLeftSlideServo.getPosition());
        telemetry.addData("Intake Slide Right Position", robot.intakeRightSlideServo.getPosition());
        telemetry.addData("Intake Turret Position", robot.intakeTurretServo.getPosition());
        telemetry.update();
    }
}
