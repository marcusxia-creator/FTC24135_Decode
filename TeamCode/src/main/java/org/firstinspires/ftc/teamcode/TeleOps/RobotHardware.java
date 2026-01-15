package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.ArrayList;


import java.util.Collections;

/*
Hardware config:
Motor:
Control hub motor:
                port 0: FR_Motor
                port 1: BR_motor




                port 2: BL_Motor
                port 3: FL_Motor
Expansion hub motor:
                port 0: Top_Shooter_Motor
                port 1: Turret_Motor
                port 2: Intake_Motor
                port 3: Bottom_Shooter_Motor

Servo:
Control hub servo:
                port 0: Empty
                port 1: Empty
                port 2: Empty
                port 3: Empty
                port 4: Empty
                port 5: Empty

Expansion hub servo:
                port 0: Spindexer_Servo
                port 1: Kicker_Servo
                port 2: Shooter_Adjuster_Servo
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
    public Servo kickerServo;
    public Servo rightSpindexerServo;
    public Servo spindexerServo;
    public Servo shooterAdjusterServo;
    public Servo leftGateServo;
    public Servo rightGateServo;
    public DcMotorEx intakeMotor;
    public DcMotorEx turretMotor;

    public DcMotorEx topShooterMotor;
    public DcMotorEx bottomShooterMotor;

    private RevHubOrientationOnRobot revHubOrientationOnRobot;

    //public ColorSensor colorSensor;// Color Sensor
    ///for debug colorSensor
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    ///public DigitalChannel limitSwitch;// Limit Switch

    public IMU imu; //IMU
    public BNO055IMU external_imu;
    public GoBildaPinpointDriver pinpoint;

    public HardwareMap hardwareMap;
    public ArrayList <VoltageSensor> voltageSensors;

    public Servo LED;

    public Limelight3A limelight3A;

    private double vEma = 12.0;                 // EMA state
    public  double vAlpha = 0.45;                // 0..1 (higher = faster response)
    public  double vMinAccept = 10.5;            // discard anything below this as junk
    public  double vDefault   = 12.0;           // fallback

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
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret_Motor");
        //Servos
        //angleServo = hardwareMap.get(Servo.class, "Angle_Servo");
        kickerServo = hardwareMap.get(Servo.class, "Kicker_Servo");
        spindexerServo = hardwareMap.get(Servo.class, "Spindexer_Servo");
        shooterAdjusterServo = hardwareMap.get(Servo.class, "Shooter_Adjuster_Servo");

        topShooterMotor = hardwareMap.get(DcMotorEx.class, "Top_Shooter_Motor");
        bottomShooterMotor = hardwareMap.get(DcMotorEx.class, "Bottom_Shooter_Motor");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        imu = hardwareMap.get(IMU.class, "imu");

        revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        LED = hardwareMap.get(Servo.class, "goBilda_LED_Light");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");

        limelight3A = hardwareMap.get(Limelight3A.class, "LimeLight3A");

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

        /// config intake motor
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /// config turret motor
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /// set run mode of shooter Motor
        topShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /** set drive motor 0 */
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        /// set spindexer servo
        spindexerServo.setDirection(Servo.Direction.REVERSE);
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

    public void initExternalIMU(){
        external_imu = hardwareMap.get(BNO055IMU.class, "external_imu");
        BNO055IMU.Parameters myBNOIMUparameters = new BNO055IMU.Parameters();
        myBNOIMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        myBNOIMUparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        myBNOIMUparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        myBNOIMUparameters.loggingEnabled      = true;
        myBNOIMUparameters.loggingTag          = "IMU";
        myBNOIMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        external_imu.initialize(myBNOIMUparameters);
    }
    public void initPinpoint() {
        pinpoint.setOffsets(92.4, -143, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
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