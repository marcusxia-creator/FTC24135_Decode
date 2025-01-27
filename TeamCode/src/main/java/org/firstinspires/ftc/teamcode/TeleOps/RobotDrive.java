package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Button Config for Drive
 * * Joy Right Y                : Drive
 * * Joy Right X                : Strafe
 * * Joy Left X                 : Turn
 * * Left Trigger               : Fine Movement + Joystick
 * * START                      : Field centric / Robot centric toggle
 * * Back                       : Reset Yaw angle
 * Gamepad 1 override Gamepad 2
 */

public class RobotDrive {

    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    private boolean startPressed = false;
    private boolean backPressed = false;

    private double powerFactor;
    private Object drive_power;

    public RobotDrive(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }

    public void Init() {
        // Initialize IMU from RobotHardware
        robot.initIMU();
    }

    @SuppressLint("DefaultLocale")
    public void DriveLoop() {
        // Toggle control mode
        if ((gamepad_1.getButton(START) || gamepad_2.getButton(START)) && !startPressed && (!gamepad_1.getButton(LEFT_BUMPER) || !gamepad_2.getButton(LEFT_BUMPER))) {
            toggleControlMode();
            debounceTimer.reset();
            startPressed = true;
        } else if (!gamepad_1.getButton(START) || !gamepad_2.getButton(START)) {
            startPressed = false;
        }

        // Reset IMU heading using button back and reset odometry
        if (gamepad_1.getButton(BACK) || gamepad_2.getButton(BACK) && !backPressed) {
            robot.initIMU();
            //robot.resetDriveEncoders();
            debounceTimer.reset();
            backPressed = true;
        } else if (!gamepad_1.getButton(BACK) || !gamepad_2.getButton(BACK)) {
            backPressed = false;
        }

        if(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.4 || gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.4){
            double factor = Math.max(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            powerFactor = RobotActionConfig.powerFactor *(1.2 - factor); //1.2 - power reduction will be (0.8 - 0.2)*power factor
        }
        else {
            powerFactor = RobotActionConfig.powerFactor;
        }
        double drive = 0.0;
        double strafe = 0.0;
        double rotate = 0.0;

        // gamepad 1 take priority override gamepad 2
        if (Math.abs(gamepad_1.getRightY()) > 0.1 || Math.abs(gamepad_1.getRightX()) > 0.1 || Math.abs(gamepad_1.getLeftX()) > 0.1) {
            drive = deadband(-gamepad_1.getRightY(),0.1);
            strafe = deadband(gamepad_1.getRightX(),0.1);
            rotate = deadband(gamepad_1.getLeftX(),0.1);
        } else if (Math.abs(gamepad_2.getRightY()) > 0.1 || Math.abs(gamepad_2.getRightX()) > 0.1 || Math.abs(gamepad_2.getLeftX()) > 0.1) {
            drive = deadband(-gamepad_2.getRightY(),0.1);
            strafe = deadband(gamepad_2.getRightX(),0.1);
            rotate = deadband(gamepad_2.getLeftX(),0.1);
        }

        // Get robot's current heading
        double currentHeading = getRobotHeading();

        // Mecanum drive calculations
        setMecanumDrivePower(drive, strafe, rotate, currentHeading, powerFactor);

        // Update telemetry with the latest data
        // empty
    }// end of driveloop


    private double getRobotHeading() {
        // Get the robot's heading from IMU
        // double heading = robot.imu().firstAngle;
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        while (heading > 180.0) {
            heading -= 360.0;
        }
        while (heading < -180.0) {
            heading += 360.0;
        }
        return -heading;
    }

    private void toggleControlMode() {
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            driveMode = DriveMode.ROBOT_CENTRIC;
        } else {
            driveMode = DriveMode.FIELD_CENTRIC;
        }
    }
    double deadband(double input, double threshold) {
        if (Math.abs(input) < threshold) { // Ignore small values
            return 0.0;
        }
        return input;
    }

    double lerp(double current, double target, double alpha) {
        return current + alpha * (target - current);
    }

    private void setMecanumDrivePower(double drive, double strafe, double rotate, double currentHeading, double powerFactor) {
        // Determine the drive mode
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            // Adjust for field-centric control using the gyro angle
            double headingRad = Math.toRadians(currentHeading);
            double temp = drive * Math.cos(headingRad) + strafe * Math.sin(headingRad);
            strafe = -drive * Math.sin(headingRad) + strafe * Math.cos(headingRad);
            drive = temp;
        }

        // Mecanum wheel drive formula
        double desiredFrontLeftPower = drive + strafe + rotate;
        double desiredFrontRightPower = drive - strafe - rotate;
        double desiredBackLeftPower = drive - strafe + rotate;
        double desiredBackRightPower = drive + strafe - rotate;

        // Constrain the power within +-1.0
        double maxPower = Math.max(
                Math.max(Math.abs(desiredFrontLeftPower), Math.abs(desiredFrontRightPower)),
                Math.max(Math.abs(desiredBackRightPower), Math.abs(desiredBackLeftPower))
        );

        if (maxPower > 1.0) {
            desiredFrontLeftPower /= maxPower;
            desiredFrontRightPower /= maxPower;
            desiredBackLeftPower /= maxPower;
            desiredBackRightPower /= maxPower;
        }

        double frontLeftPower = lerp(robot.frontLeftMotor.getPower(), desiredFrontLeftPower, RobotActionConfig.slowness);
        double frontRightPower = lerp(robot.frontRightMotor.getPower(), desiredFrontRightPower,  RobotActionConfig.slowness);
        double backLeftPower = lerp(robot.backLeftMotor.getPower(), desiredBackLeftPower,  RobotActionConfig.slowness);
        double backRightPower = lerp(robot.backRightMotor.getPower(), desiredBackRightPower,  RobotActionConfig.slowness);


        // Set motor powers
        robot.frontLeftMotor.setPower(Range.clip(frontLeftPower * powerFactor, -1.0, 1.0));
        robot.frontRightMotor.setPower(Range.clip(frontRightPower * powerFactor, -1.0, 1.0));
        robot.backLeftMotor.setPower(Range.clip(backLeftPower * powerFactor, -1.0, 1.0));
        robot.backRightMotor.setPower(Range.clip(backRightPower * powerFactor, -1.0, 1.0));

    }

    // Method to get left encoder count
    /*
    public int [] getEncoderCounts() {
        int[] counts = new int[3];
        counts[0] = robot.leftodometry.getCurrentPosition();
        counts[1] = robot.rightodometry.getCurrentPosition();
        counts[2] = robot.centerodometry.getCurrentPosition();
        return counts;
    }
    */
    public double[] getVelocity() {
        double[] velocities = new double[4];
        velocities[0] = robot.frontLeftMotor.getVelocity();
        velocities[1] = robot.frontRightMotor.getVelocity();
        velocities[2] = robot.backLeftMotor.getVelocity();
        velocities[3] = robot.backRightMotor.getVelocity();
        return velocities;
    }

    public enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }
    public DriveMode getDriveMode() {
        return driveMode;
    }
}
