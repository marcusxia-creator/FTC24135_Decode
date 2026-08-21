package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;


import java.util.Collections;

/*
Hardware config:
Motor:
Control hub motor:
                port 0: BR_motor
                port 1: BL_Motor
                port 2: FL_Motor
                port 3: FR_Motor
Expansion hub motor:
                port 0: Turret_Motor
                port 1: Intake_Motor
                port 2: Bottom_Shooter_Motor
                port 3: Top_Shooter_Motor

Servo:
Control hub servo:
                port 0: Empty
                port 1: Empty
                port 2: Empty
                port 3: Empty
                port 4: Empty
                port 5: Kicker_Servo

Expansion hub servo:
                port 0: Spindexer_Servo
                port 1: Shooter_Adjuster_Servo
                port 2: Empty
                port 3: goBilda_LED_Light
                port 4: Empty
                port 5: Empty

I2C port
EXP hub:
                port 0: external_imu
Control hub:
                port 0: control hub imu
                port 1: Pinpoint (odometry computer)
                port 2: Empty
                port 3: Color_Sensor
Digital Port
Control hub
                port 7: Empty

 */

public class RobotHardware {
    //motors
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx intakeMotor;

    //public ColorSensor colorSensor;// Color Sensor
    ///for debug colorSensor

    //Legacy colour sensors, kept for auto errors

    ///public DigitalChannel limitSwitch;// Limit Switch

    public IMU imu; //IMU

    public HardwareMap hardwareMap;
    public ArrayList <VoltageSensor> voltageSensors;
    private List<LynxModule> allHubs;

    private double vEma = 12.0;                 // EMA state
    public  double vAlpha = 0.45;                // 0..1 (higher = faster response)
    public  double vMinAccept = 10.5;            // discard anything below this as junk
    public  double vDefault   = 12.0;           // fallback

    // Throttle for getBatteryVoltageRobust(): voltage sensor reads are live,
    // blocking hub commands (not covered by clearBulkCache()), and battery
    // voltage doesn't need re-reading every loop tick.
    private final List<Double> voltageReadBuffer = new ArrayList<>();
    private long lastVoltageReadMs = 0;
    private static final long VOLTAGE_READ_INTERVAL_MS = 250;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap; // store the hardwareMap reference
        /**Set up motors**/
    }


    public void init() {
        //Drive train motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");

        voltageSensors = new ArrayList<>(hardwareMap.getAll(VoltageSensor.class));
        /// Reset the drive motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /// Set drive motor run mode
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //set motor mode
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        /// config drive motor set front left motor reverse
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /// brake instead of coast when drive power hits 0
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /// config intake motor
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        /** set drive motor 0 */
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
        long now = System.currentTimeMillis();
        if (now - lastVoltageReadMs < VOLTAGE_READ_INTERVAL_MS) {
            return vEma;   // reuse last reading — voltage sensor reads are live hub commands
        }
        lastVoltageReadMs = now;

        voltageReadBuffer.clear();
        for (VoltageSensor vs : voltageSensors) {
            double v = vs.getVoltage();
            if (v > vMinAccept) voltageReadBuffer.add(v);   // keep plausible readings only
        }
        double vMed = voltageReadBuffer.isEmpty() ? vDefault : median(voltageReadBuffer);
        // EMA smoothing
        vEma = vAlpha * vMed + (1.0 - vAlpha) * vEma;
        return vEma;
    }

    //bulk reading
    public void initializeBulkReading(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }
    }
}