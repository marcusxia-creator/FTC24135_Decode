package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;

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

    public WebcamName webcam1;


    //Intake servos

    //public ColorSensor colorSensor;// Color Sensor
    ///for debug colorSensor
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    ///public DigitalChannel limitSwitch;// Limit Switch

    public IMU imu; //IMU
    public HardwareMap hardwareMap;
    public ArrayList <VoltageSensor> voltageSensors;
    public GoBildaPinpointDriver odo;

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

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Webcam
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

                // limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        voltageSensors = new ArrayList<>(hardwareMap.getAll(VoltageSensor.class));
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
        // set robot motor power 0
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");




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
