package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotMovement {
    private DriveTrainControlMode driveTrainControlMode;

    //Declare gamepad
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;

    //Declare robot
    private final RobotHardware robot;

    //Declare constructor
    public RobotMovement (RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.driveTrainControlMode = DriveTrainControlMode.ROBOT_CENTRIC;
    }

    //Set the debounce timer
    private final ElapsedTime debounceTimer = new ElapsedTime();

    public void robotDriveTrain() {

        //Set the imu parameters
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double x = 0;
        double y = 0;
        double rx = 0;

        //Declare the motor max speed
        double motorMaxSpeed = RobotActionConfig.drivetrain_Power_Factor;

        //Set the gamepad parameters
        if (Math.abs(gamepad_1.getRightY()) > 0.1 || Math.abs(gamepad_1.getRightX()) > 0.1 || Math.abs(gamepad_1.getLeftX()) > 0.1) {
            y = -gamepad_1.getRightY();
            x = gamepad_1.getRightX();
            rx = gamepad_1.getLeftX();
        } else if (Math.abs(gamepad_2.getRightY()) > 0.1 || Math.abs(gamepad_2.getRightX()) > 0.1 || Math.abs(gamepad_2.getLeftX()) > 0.1) {
            y = -gamepad_2.getRightY();
            x = gamepad_2.getRightX();
            rx = gamepad_2.getLeftX();
        }

        if ((gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.6) || (gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.6)) {
            motorMaxSpeed /= 2;
        }

        /**Future note, check if short circuit is needed**/
        if ((gamepad_1.getButton(GamepadKeys.Button.START) && debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) || (gamepad_2.getButton(GamepadKeys.Button.START) && debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD)) {
            debounceTimer.reset();
            if (driveTrainControlMode == DriveTrainControlMode.ROBOT_CENTRIC) {
                driveTrainControlMode = DriveTrainControlMode.FIELD_CENTRIC;
            }
            else {
                driveTrainControlMode = DriveTrainControlMode.ROBOT_CENTRIC;
            }
        }

        if ((gamepad_1.getButton(GamepadKeys.Button.BACK) && debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) || (gamepad_2.getButton(GamepadKeys.Button.BACK) && debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD)) {
            debounceTimer.reset();
            robot.imu.resetYaw();
        }

        if (driveTrainControlMode == DriveTrainControlMode.ROBOT_CENTRIC) {
            //Robot Centric drive power calculations
            double robotCentricDenominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double robotCentricFrontLeftMotorPower = ((y + x + rx) * motorMaxSpeed) / robotCentricDenominator;
            double robotCentricBackLeftMotorPower = ((y - x + rx) * motorMaxSpeed) / robotCentricDenominator;
            double robotCentricFrontRightMotorPower = ((y - x - rx) * motorMaxSpeed) / robotCentricDenominator;
            double robotCentricBackRightMotorPower = ((y + x - rx) * motorMaxSpeed) / robotCentricDenominator;

            //Set the motor power
            robot.frontLeftMotor.setPower(robotCentricFrontLeftMotorPower);
            robot.backLeftMotor.setPower(robotCentricBackLeftMotorPower);
            robot.frontRightMotor.setPower(robotCentricFrontRightMotorPower);
            robot.backRightMotor.setPower(robotCentricBackRightMotorPower);
        }

        else {
            //Field Centric drive power calculations
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double fieldCentricDenominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double fieldCentricFrontLeftMotorPower = ((rotY + rotX + rx) * motorMaxSpeed) / fieldCentricDenominator;
            double fieldCentricBackLeftMotorPower = ((rotY - rotX + rx) * motorMaxSpeed) / fieldCentricDenominator;
            double fieldCentricFrontRightMotorPower = ((rotY - rotX - rx) * motorMaxSpeed) / fieldCentricDenominator;
            double fieldCentricBackRightMotorPower = ((rotY + rotX - rx) * motorMaxSpeed) / fieldCentricDenominator;

            robot.frontLeftMotor.setPower(fieldCentricFrontLeftMotorPower);
            robot.backLeftMotor.setPower(fieldCentricBackLeftMotorPower);
            robot.frontRightMotor.setPower(fieldCentricFrontRightMotorPower);
            robot.backRightMotor.setPower(fieldCentricBackRightMotorPower);
        }
    }

    //Declare the enum
    private enum DriveTrainControlMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
}
