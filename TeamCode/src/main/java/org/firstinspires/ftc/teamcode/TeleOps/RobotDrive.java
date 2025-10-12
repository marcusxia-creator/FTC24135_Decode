package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler;

/** Button Config for Drive
 * * Joy Right Y                : Drive
 * * Joy Right X                : Strafe
 * * Joy Left X                 : Turn
 * * Left Trigger               : Fine Movement + Joystick
 * * START                      : Field centric / Robot centric toggle
 * * Back                       : Reset Yaw angle --- removed
 * Gamepad 1 override Gamepad 2
 */

public class RobotDrive {

    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    private boolean startPressed = false;
    private boolean backPressed = false;

    private double powerFactor;
    private Object drive_power;

    private double rotate_Slowness = 0.75;

    public IceWaddler iceWaddler;

    public FSMAprilTagProc AprilTag;

    public boolean autoHeading = false;

    public RobotDrive(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }

    public void Init(FSMAprilTagProc AprilTag) {
        // Initialize IMU from RobotHardware
        iceWaddler = new IceWaddler(robot);

        iceWaddler.Init(IceWaddler.CONTROLMODE.POWER,
                new Pose2D(DistanceUnit.METER,0,0, AngleUnit.DEGREES,0),
                false);

        this.AprilTag = AprilTag;
    }


    @SuppressLint("DefaultLocale")
    public void DriveLoop() {

        /** Reset IMU heading using button back and reset odometry
         * * need to remove this feature, as back button reseting imu may interfer with Roadrunner pose estimation.
         * * back button is using for retracting slide after auto phase and initial start of teleops

        if (gamepad_1.getButton(BACK) || gamepad_2.getButton(BACK) && !backPressed) {
            //robot.initIMU();
            //robot.resetDriveEncoders();
            debounceTimer.reset();
            backPressed = true;
        } else if (!gamepad_1.getButton(BACK) || !gamepad_2.getButton(BACK)) {
            backPressed = false;
        }
         */

        if(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.4 || gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.4){
            double factor = Math.max(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            powerFactor = Range.clip(RobotActionConfig.powerFactor *(1.4 - factor),0,1); //1.0 - power reduction will be 0.6 - 0.2
        }
        else {
            powerFactor = RobotActionConfig.powerFactor;
        }
        double drive = 0.0;
        double strafe = 0.0;
        double rotate = 0.0;

        // gamepad 1 take priority override gamepad 2
        if (Math.abs(gamepad_1.getRightY()) > 0.1 || Math.abs(gamepad_1.getRightX()) > 0.1 || Math.abs(gamepad_1.getLeftX()) > 0.1) {
            drive = deadband(-gamepad_1.getRightY(),0.1) * RobotActionConfig.powerFactor;
            strafe = deadband(gamepad_1.getRightX(),0.1) * RobotActionConfig.powerFactor;
            rotate = deadband(gamepad_1.getLeftX(),0.1) * rotate_Slowness;
        } else if (Math.abs(gamepad_2.getRightY()) > 0.1 || Math.abs(gamepad_2.getRightX()) > 0.1 || Math.abs(gamepad_2.getLeftX()) > 0.1) {
            drive = deadband(-gamepad_2.getRightY(),0.1) * RobotActionConfig.powerFactor;
            strafe = deadband(gamepad_2.getRightX(),0.1) * RobotActionConfig.powerFactor;
            rotate = deadband(gamepad_2.getLeftX(),0.1) * rotate_Slowness;
        }

        if(autoHeading){
            AprilTag.procState = FSMAprilTagProc.ProcState.RUNNING;
            if(AprilTag.tag != null) {
                rotate = RobotActionConfig.autoHeadingCoeff*AprilTag.tag.center.x;
            }
        }
        else{
            AprilTag.procState = FSMAprilTagProc.ProcState.STOPPED;
        }

        // Toggle Auto Heading
        if ((gamepad_1.getButton(START) || gamepad_2.getButton(START)) && !startPressed && (!gamepad_1.getButton(LEFT_BUMPER) || !gamepad_2.getButton(LEFT_BUMPER))) {
            autoHeading = !autoHeading;
            debounceTimer.reset();
            startPressed = true;
        } else if (!gamepad_1.getButton(START) || !gamepad_2.getButton(START)) {
            startPressed = false;
        }

        // Mecanum drive calculations
        iceWaddler.runByPower(new Pose2D(DistanceUnit.METER, drive, strafe, AngleUnit.DEGREES, 0), rotate);
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
    double deadband(double input, double threshold) {
        if (Math.abs(input) < threshold) { // Ignore small values
            return 0.0;
        }
        return input;
    }

    double lerp(double current, double target, double accel_Slowness, double decel_Slowness) {
        double alpha = (Math.abs(target) > Math.abs(current)) ? accel_Slowness : decel_Slowness;
        return current + alpha * (target - current);
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
}
