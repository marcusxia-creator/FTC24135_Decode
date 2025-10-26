package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
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

    public DcMotorEx intakeMotor;// Vertical Slide Motor
    public DcMotorEx shooterMotor;// Vertical Slide Motor

    public Servo pushRampServo;
    public Servo spindexerServo;
    public Servo leftGateServo;
    public Servo rightGateServo;
    //limit switch
    public DigitalChannel limitSwitch;

    ///ColorSensor
    public ColorSensor colorSensor;

    public DistanceSensor distanceSensor;
    ///public DigitalChannel limitSwitch;// Limit Switch

    /// goBilda LED light
    public Servo rgbLED;

    /// goBilda Pinpoint Driver
    public GoBildaPinpointDriver pinpointDriver;
    public Pose2D pinpointPose;

    ///IMU

    public IMU imu; //IMU

    public HardwareMap hardwareMap;
    public ArrayList <VoltageSensor> voltageSensors;

    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    private double vEma = 12.0;                 // EMA state
    public  double vAlpha = 0.45;                // 0..1 (higher = faster response)
    public  double vMinAccept = 10.5;            // discard anything below this as junk
    public  double vDefault   = 12.0;           // fallback

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {

        /**Set up motors**/
        //Drive train motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");
        //Intake and shooter servos
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter_Motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");
        //Servos
        //angleServo = hardwareMap.get(Servo.class, "Angle_Servo");
        pushRampServo = hardwareMap.get(Servo.class, "Ramp_Servo");
        spindexerServo = hardwareMap.get(Servo.class, "Spindexer_Servo");
        leftGateServo = hardwareMap.get(Servo.class, "Left_Gate_Servo");
        rightGateServo = hardwareMap.get(Servo.class, "Right_Gate_Servo");

        leftGateServo.setDirection(Servo.Direction.REVERSE);
        rightGateServo.setDirection(Servo.Direction.FORWARD);
        //color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Color_Sensor");

        //limit switch
        //limitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch");
        //limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        //goBilda LED light
        rgbLED = hardwareMap.get(Servo.class, "goBilda_LED_Light");

        //goBilda Pinpoint Driver
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");

        //voltage sensor
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

        //Set run mode of intake and shooter motors
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
                        RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                ));
        imu.initialize(myIMUparameters);
        imu.resetYaw();
    }

    public void initPinPoint() {
        pinpointDriver.setOffsets(-149.225, -165.1,DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.resetPosAndIMU();
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
