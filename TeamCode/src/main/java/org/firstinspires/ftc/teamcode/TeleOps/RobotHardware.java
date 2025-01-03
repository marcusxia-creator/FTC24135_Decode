package org.firstinspires.ftc.teamcode.TeleOps;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

/**
Hardware config:
Motor:
Control hub motor:
 Port 0: FL_Motor,
 Port 1: BL_motor,
 Port 2: FR_Motor,
 Port 3: BR_Motor

Control hub servo:
 Port 0: Intake_Wrist_Servo
 Port 1: Intake_Arm_Left_Servo
 Port 2: Intake_Slide_Left_Servo
 Port 3: Deposit_Claw_Servo
 Port 4: Deposit_Arm_Servo
 Port 5: Nothing

Expansion hub motor:
 Port 0: VS_Motor_Left
 Port 1: VS_Motor_Right

Expansion hub servo:
 Port 0: Nothing
 Port 1: Intake_Slide_Right_Servo
 Port 2: Deposit_Wrist_Servo
 Port 3: Intake_Claw_Servo
 Port 4: Intake_Rotation_Servo
 Port 5: Intake_Arm_Right_Servo

Control hub I2C:
 Port 1: PinPoint Odometry
 Port 2: Color_Sensor

 **/

public class RobotHardware {

    //Drive chassis motor
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    public DcMotorEx liftMotorLeft;// Vertical Slide Motor
    public DcMotorEx liftMotorRight;// Vertical Slide Motor

    //Intake servos
    public Servo intakeLeftSlideServo;
    public Servo intakeRightSlideServo;
    public Servo intakeLeftArmServo;
    public Servo intakeRightArmServo;
    public Servo intakeWristServo;
    public Servo intakeRotationServo;
    public Servo intakeClawServo;

    //Deposit servos
    public Servo depositArmServo;
    public Servo depositWristServo;
    public Servo depositClawServo;

    //Color sensor
    public ColorSensor colorSensor;

    public IMU imu; //IMU

    public HardwareMap hardwareMap;

    public void init(@NonNull HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;

        /**Set up motors**/
        //Drive train motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");
        //Lift motors
        liftMotorLeft = hardwareMap.get(DcMotorEx.class,"VS_Left_Motor");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "VS_Right_Motor");


        /**set servos**/
        //Intake servo
        intakeLeftSlideServo = hardwareMap.get(Servo.class, "Intake_Slide_Left_Servo");
        intakeRightSlideServo = hardwareMap.get(Servo.class, "Intake_Slide_Right_Servo");
        intakeLeftArmServo = hardwareMap.get(Servo.class, "Intake_Arm_Left_Servo");
        intakeRightArmServo = hardwareMap.get(Servo.class, "Intake_Arm_Right_Servo");
        intakeWristServo = hardwareMap.get(Servo.class, "Intake_Wrist_Servo");
        intakeRotationServo = hardwareMap.get(Servo.class, "Intake_Rotation_Servo");
        intakeClawServo = hardwareMap.get(Servo.class, "Intake_Claw_Servo");
        //Deposit servo
        depositArmServo = hardwareMap.get(Servo.class, "Deposit_Arm_Servo");
        depositWristServo = hardwareMap.get(Servo.class, "Deposit_Wrist_Servo");
        depositClawServo = hardwareMap.get(Servo.class, "Deposit_Claw_Servo");
        //Color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");

        //set motor mode and motor direction
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);  // Reverse the left motor if needed
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);  // Reverse the left motor if needed

        //Reset the drive train motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set drive train motor run mode
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //set motor mode
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode

        //Set intake slide servo direction
        intakeLeftSlideServo.setDirection(Servo.Direction.REVERSE);

        //set servo direction - intake and deposit
        intakeRightArmServo.setDirection(Servo.Direction.REVERSE);
        intakeRightSlideServo.setDirection(Servo.Direction.REVERSE);

        //set slide motors to RUN_TO_POSITION for vertical slide motor
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Reset the motor encoder
        liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Set the run mode of the motors
        liftMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // set robot motor power 0
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }// End of init

    // Initialize IMU
    public void initIMU() {
        // set up REV imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                ));
        imu.initialize(myIMUparameters);
    }
}
