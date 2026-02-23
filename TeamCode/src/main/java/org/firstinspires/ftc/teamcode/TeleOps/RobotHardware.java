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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
                port 5: Empty

Expansion hub servo:
                port 0: Spindexer_Servo
                port 1: Shooter_Adjuster_Servo
                port 2: Kicker_Servo
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
    public Servo spindexerServo;
    public Servo shooterAdjusterServo;
    public DcMotorEx intakeMotor;
    public DcMotorEx turretMotor;

    public DcMotorEx topShooterMotor;
    public DcMotorEx bottomShooterMotor;

    private RevHubOrientationOnRobot revHubOrientationOnRobot;

    //public ColorSensor colorSensor;// Color Sensor
    ///for debug colorSensor
    public ColorSensor colorSensor;
    public DistanceSensor distanceSensor;

    public DigitalChannel limitSwitch;// Limit Switch

    public IMU imu; //IMU
    public BNO055IMU external_imu;
    // NEW: Variable to store the initial IMU heading offset for external IMU
    private double externalImuHeadingOffset = 0;

    public GoBildaPinpointDriver pinpoint;

    public HardwareMap hardwareMap;
    public ArrayList <VoltageSensor> voltageSensors;

    public Servo LED;

    public Limelight3A limelight;

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
        external_imu = hardwareMap.get(BNO055IMU.class, "external_imu");

        LED = hardwareMap.get(Servo.class, "goBilda_LED_Light");
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Color_Sensor");

        limitSwitch = hardwareMap.get(DigitalChannel .class, "Limit_Switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        limelight = hardwareMap.get(Limelight3A.class, "LimeLight3A");

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
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //turretMotor.setTargetPositionTolerance(3);

        /// set run mode of shooter Motor
        topShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ///set servo direction
        shooterAdjusterServo.setDirection(Servo.Direction.REVERSE);
        kickerServo.setDirection(Servo.Direction.REVERSE);


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

    public void initExternalIMU(){
        BNO055IMU.Parameters myBNOIMUparameters = new BNO055IMU.Parameters();
        myBNOIMUparameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        myBNOIMUparameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        myBNOIMUparameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        myBNOIMUparameters.loggingEnabled       = true;
        myBNOIMUparameters.loggingTag           = "IMU";
        myBNOIMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        external_imu.initialize(myBNOIMUparameters);
        // --- NEW: Reset external IMU yaw to a desired 'zero' ---
        // Call this *after* the IMU has been initialized.
        // If you want the IMU to *report* 90 degrees when facing forward (its default 0),
        // you would subtract its initial reading from 90.
        // If you want to make its current heading *become* 90 degrees, you'd calculate the difference.
        // For simplicity, let's assume you want its *current* forward-facing direction to be 90 degrees.
        resetExternalImuYaw(90); // Call this to set the "zero" of the IMU to 90
    }
    /**
     * Reads the current raw yaw from the external BNO055 IMU.
     * @return Current yaw angle in degrees (-180 to 180).
     */
    public double getExternalImuRawYaw() {
        Orientation angles = external_imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; // 'firstAngle' corresponds to Yaw in ZYX order
    }

    /**
     * Resets the external IMU's reported yaw to a specified value (e.g., 0 or 90).
     * This calculates an offset so subsequent calls to getExternalImuYaw()
     * start from this specified angle.
     * @param targetYaw The desired yaw angle (e.g., 0.0 or 90.0) for the current robot orientation.
     */
    public void resetExternalImuYaw(double targetYaw) {
        // Calculate the difference between the target yaw and the current raw yaw.
        // This offset will be added to future readings.
        // If current raw is 5 and target is 0, offset is -5.
        // If current raw is 5 and target is 90, offset is 85.
        externalImuHeadingOffset = targetYaw - getExternalImuRawYaw();
    }

    /**
     * Gets the external IMU's yaw angle, adjusted by the stored offset.
     * @return The adjusted yaw angle in degrees.
     */
    public double getExternalImuYaw() {
        double rawYaw = getExternalImuRawYaw();
        double adjustedYaw = rawYaw + externalImuHeadingOffset;

        // Optional: Normalize the angle to be within -180 to 180 degrees
        // (BNO055IMU usually reports this by default, but good practice for sums)
        if (adjustedYaw > 180) {
            adjustedYaw -= 360;
        } else if (adjustedYaw <= -180) {
            adjustedYaw += 360;
        }
        return adjustedYaw;
    }

    public void initPinpoint() {
        pinpoint.setOffsets(38.1, -184.15, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //pinpoint.resetPosAndIMU();// disable the pose and imu reset due to the pose2d transfer consideration
    }

    public void turretInit() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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