package org.firstinspires.ftc.teamcode.TeleOps;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@TeleOp (name = "Servo Test", group = "org.firstinspires.ftc.teamcode")
public class ServoTest extends OpMode{

    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private RobotHardware robot;

    //declear initial servo position
    private double servoposition;
    private double servoposition2;
    int iniPosition = 50;
    int delta_Position = 50;
    int current_Position;
    private static final double speed = 0.4;

    double deadband(double input, double threshold) {
        if (Math.abs(input) < threshold) { // Ignore small values
            return 0.0;
        }
        return input;
    }

   //
    private final ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private static final double DEBOUNCE_THRESHOLD = 0.25;

    public ServoTest(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }
    @Override
    public void init(){
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        robot = new RobotHardware(hardwareMap);

        robot.depositLeftArmServo.setPosition(0.0);
        robot.depositRightArmServo.setPosition(0.0);
        robot.depositWristServo.setPosition(0.0);
        robot.depositClawServo.setPosition(0.0);

        robot.intakeArmServo.setPosition(0.0);
        robot.intakeLeftSlideServo.setPosition(0.03);
        robot.intakeRightSlideServo.setPosition(0.03);
        robot.intakeWristServo.setPosition(0.0);
        robot.intakeRotationServo.setPosition(0.46);
        robot.intakeClawServo.setPosition(0.5);

        robot.liftMotorLeft.setTargetPosition(iniPosition);
        robot.liftMotorRight.setTargetPosition(iniPosition);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
    }
    @Override
    public void loop() {
        /*deposit system setup*/
        // A for claw open
        if (gamepad_1.getButton(A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();

            //use to be 0.01
            servoposition = robot.depositClawServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.depositClawServo.setPosition(Range.clip(servoposition,0,1));
        }
        //B for claw close
        if (gamepad_1.getButton(B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            //use to be 0.01
            servoposition = robot.depositClawServo.getPosition();
            servoposition -= 0.01;
            robot.depositClawServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositLeftArmServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.depositLeftArmServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositRightArmServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            robot.depositRightArmServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition,0,1));
        }

        if (gamepad_1.getButton(X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {

            current_Position = robot.liftMotorLeft.getCurrentPosition();

            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position + delta_Position,50,3500));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position + delta_Position,50,3500));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }
        if (gamepad_1.getButton(Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {

            current_Position = robot.liftMotorLeft.getCurrentPosition();

            robot.liftMotorLeft.setTargetPosition(Range.clip(current_Position - delta_Position,50,3500));
            robot.liftMotorRight.setTargetPosition(Range.clip(current_Position - delta_Position,50,3500));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }

        /* Intake System */
        // gamepad2 DPAD_UP button for arm rise
        if (gamepad_2.getButton(DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();

            //use to be 0.01
            servoposition = robot.intakeArmServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.intakeArmServo.setPosition(Range.clip(servoposition,0,1));
        }
        // gamepad2 DPAD_DOWN for arm lower
        if (gamepad_2.getButton(DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            //use to be 0.01
            servoposition = robot.intakeArmServo.getPosition();
            servoposition -= 0.01;
            robot.intakeArmServo.setPosition(Range.clip(servoposition,0,1));
        }
        // gamepad2 A button for claw open
        if (gamepad_2.getButton(A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeClawServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.intakeClawServo.setPosition(Range.clip(servoposition,0,1));
        }
        // gamepad2 B buttion for claw close
        if (gamepad_2.getButton(B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeClawServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            robot.intakeClawServo.setPosition(Range.clip(servoposition,0,1));
        }

        // gamepad2 DPAD_LEFT for intake wrist servo
        if (gamepad_2.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition();
            //servoposition2 = robot.intakeLeftSlideServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            //servoposition2 += 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition,0,1));
            //robot.intakeLeftSlideServo.setPosition(Range.clip(servoposition2,0,1));
            //robot.intakeRightSlideServo.setPosition(Range.clip(servoposition2,0,1));
        }

        // gamepad2 DPAD_RIGHT for intake wrist servo
        if (gamepad_2.getButton(DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition();
            //servoposition2 = robot.intakeLeftSlideServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            //servoposition2 -= 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition, 0, 1));
            //robot.intakeLeftSlideServo.setPosition(Range.clip(servoposition2,0,1));
            //robot.intakeRightSlideServo.setPosition(Range.clip(servoposition2,0,1));
        }

        //gamepad2 X for intake claw rotation

        // gamepad2 DPAD_RIGHT for intake wrist servo
        if (gamepad_2.getButton(X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeRotationServo.getPosition();
            //use to be 0.01
            servoposition += 0.01;
            robot.intakeRotationServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        // gamepad2 DPAD_RIGHT for intake wrist servo
        if (gamepad_2.getButton(Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeRotationServo.getPosition();
            //use to be 0.01
            servoposition -= 0.01;
            robot.intakeRotationServo.setPosition(Range.clip(servoposition, 0, 1));
        }
    }
    public void DriveTest (){
        double rotate_Slowness = 0.75;
        double drive = 0.0;
        double strafe = 0.0;
        double rotate = 0.0;

        if (Math.abs(gamepad_1.getRightY()) > 0.1 ||
            Math.abs(gamepad_1.getRightX()) > 0.1 ||
            Math.abs(gamepad_1.getLeftX()) > 0.1) {

            drive = deadband(-gamepad_1.getRightY(), 0.1);
            strafe = deadband(gamepad_1.getRightX(), 0.1);
            rotate = deadband(gamepad_1.getLeftX() * rotate_Slowness, 0.1);

            double frontLeft = drive + strafe + rotate;
            double frontRight = drive - strafe - rotate;
            double backLeft = drive - strafe + rotate;
            double backRight = drive + strafe - rotate;

            robot.frontLeftMotor.setPower(frontLeft);
            robot.frontRightMotor.setPower(frontRight);
            robot.backLeftMotor.setPower(backLeft);
            robot.backRightMotor.setPower(backRight);
        } else {
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

        }

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
        telemetry.addData("Intake Slide Position", robot.intakeLeftSlideServo.getPosition());
        telemetry.addData("Intake Slide Position", robot.intakeRightSlideServo.getPosition());
        telemetry.update();

    }

}


