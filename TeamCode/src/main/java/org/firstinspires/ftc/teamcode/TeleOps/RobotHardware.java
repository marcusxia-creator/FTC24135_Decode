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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.List;
import java.util.ArrayList;

import java.util.ArrayList;
import java.util.Collections;

/*
Hardware config:
Motor:
Control hub motor:
                port 0: FL_Motor
                port 1: BL_motor
                port 2: FR_Motor
                port 3: BR_Motor
Expansion hub motor:
                port 0: VS_Left_Motor
                port 3: VS_Right_Motor
                port 1: par (encoder for odometry pod in X direction - parallel direction)
                port 2: perp (encoder for odometry pod in Y direction - perpendicular direction)

Servo:
EXP hub:
                port 3: Intake_Wrist_Servo
                port 5: Intake_Arm_Left_Servo
                port 0: Deposit_Wrist_Servo
                port 1: Deposit_Claw_Servo
                port 2: Deposit_Arm_Servo
                port 4: Empty

Control hub:
                port 0: Empty
                port 1: Intake_Slide_Right_Servo
                port 2: Intake_Slide_Left_Servo
                port 3: Intake_Claw_Servo
                port 4: Intake_Rotation_Servo
                port 5: Intake_Arm_Right_Servo


I2C port
Control hub
                port 0: control hub imu
                port 1: Pinpoint (odometry computer)
                port 2: Color_Sensor
Digital Port
Control hub
                port 7: LimitSwitch

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
    public ArrayList <VoltageSensor> voltageSensors;


    private double vEma = 12.0;                 // EMA state
    public  double vAlpha = 0.45;                // 0..1 (higher = faster response)
    public  double vMinAccept = 10.5;            // discard anything below this as junk
    public  double vDefault   = 12.0;           // fallback

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

        voltageSensors = new ArrayList<>(hardwareMap.getAll(VoltageSensor.class));

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
        liftMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

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

    private static double median(List<Double> xs) {
        Collections.sort(xs);
        int n = xs.size();
        return n == 0 ? Double.NaN : (n % 2 == 1 ? xs.get(n/2) : 0.5*(xs.get(n/2-1)+xs.get(n/2)));
    }

    public double getBatteryVoltageRobust() {
        List<Double> vals = new ArrayList<>();
        for (VoltageSensor vs : voltageSensors) {
            double v = vs.getVoltage();
            if (v > vMinAccept) vals.add(v);        // keep plausible readings only
        }
        double vMed = vals.isEmpty() ? vDefault : median(vals);
        // EMA smoothing
        vEma = vAlpha * vMed + (1.0 - vAlpha) * vEma;
        return vEma;
    }
}
