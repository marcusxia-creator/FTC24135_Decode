package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;

/*
Hardware configuration:
Motors:
Control hub motors:
    0 FR_Motor
    1 BR_Motor
    2 FL_Motor
    3 BL_Motor
Expansion hub motors:
    0 RS_Motor
    1 -----
    2 -----
    3 LS_Motor
Servos:
Control hub servos:
    0 Intake_Slide_Left_Servo
    1 Intake_Slide_Right_Servo
    2 -----
    3 Intake_Rotation Servo
    4 Intake_Claw_Servo
    5 Intake_Wrist_Servo
Expansion hub servos:
    0 Deposit_Left_Arm_Servo
    1 Deposit-Right_Arm_Servo
    2 Deposit_Wrist_Horrible
    3 Deposit_Claw_Servo
    4 Intake_Arm_Servo
    5 Intake_Turret_Servo
Control hub I2C Bus 0: imu
Control hub I2C Bus 1: Pinpoint
*/


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
    public Servo intakeArmServo;
    public Servo intakeTurretServo;
    public Servo intakeRotationServo;
    public Servo intakeClawServo;
    public Servo intakeWristServo;

    //Deposit servos
    public Servo depositLeftArmServo;
    public Servo depositRightArmServo;
    public Servo depositWristServo;
    public Servo depositClawServo;

    //public ColorSensor colorSensor;// Color Sensor
    ///for debug colorSensor
    public NormalizedColorSensor colorSensor;

    ///public DigitalChannel limitSwitch;// Limit Switch

    public IMU imu; //IMU
    public HardwareMap hardwareMap;

    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    public RobotHardware(HardwareMap hardwareMap) {
    }


    public void init(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap; // store the hardwareMap reference
        /**Set up motors**/
        //Drive train motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");
        //Lift motors
        liftMotorLeft = hardwareMap.get(DcMotorEx.class,"LS_Motor");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "RS_Motor");


        /**set servos**/
        //Intake servo
        intakeArmServo = hardwareMap.get(Servo.class, "Intake_Arm_Servo");
        intakeLeftSlideServo = hardwareMap.get(Servo.class, "Intake_Slide_Left_Servo");
        intakeRightSlideServo = hardwareMap.get(Servo.class, "Intake_Slide_Right_Servo");
        intakeWristServo = hardwareMap.get(Servo.class, "Intake_Wrist_Servo");
        intakeRotationServo = hardwareMap.get(Servo.class, "Intake_Rotation_Servo");
        intakeClawServo = hardwareMap.get(Servo.class, "Intake_Claw_Servo");
        intakeTurretServo = hardwareMap.get(Servo.class, "Intake_Turret_Servo");
        //Deposit servo
        depositLeftArmServo = hardwareMap.get(Servo.class, "Deposit_Left_Arm_Servo");
        depositRightArmServo = hardwareMap.get(Servo.class, "Deposit_Right_Arm_Servo");
        depositWristServo = hardwareMap.get(Servo.class, "Deposit_Wrist_Servo");
        depositClawServo = hardwareMap.get(Servo.class, "Deposit_Claw_Servo");
        //Color sensor
        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "Color_Sensor");
        //colorSensor.setGain(2);
        //colorSensor.enableLed(true); // this is for Non normalized colorSensor.
        //Limit Switch
        //limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch");
       // limitSwitch.setMode(DigitalChannel.Mode.INPUT);

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

        //set servo direction - intake and deposit
        intakeRightSlideServo.setDirection(Servo.Direction.REVERSE);
        intakeWristServo.setDirection(Servo.Direction.REVERSE);
        depositLeftArmServo.setDirection(Servo.Direction.REVERSE);

        //set slide motors direction
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                ));
        imu.initialize(myIMUparameters);
        imu.resetYaw();
    }

    public void initPinPoint() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-149.225, -165.1); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
}
