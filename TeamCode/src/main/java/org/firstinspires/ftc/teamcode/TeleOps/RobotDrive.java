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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler;

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

    private final GamepadComboInput gamepadComboInput;
    private final RobotHardware robot;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    private boolean startPressed = false;
    private boolean fieldCentric = false;

    private double powerFactor;
    private Object drive_power;

    private double rotate_Slowness = 0.75;

    private final SlewRateLimiter frontLeftLimiter = new SlewRateLimiter(0.2,0.5);
    private final SlewRateLimiter frontRightLimiter = new SlewRateLimiter(0.2,0.5);
    private final SlewRateLimiter backLeftLimiter = new SlewRateLimiter(0.2,0.5);
    private final SlewRateLimiter backRightLimiter = new SlewRateLimiter(0.2,0.5);

    public RobotDrive(RobotHardware robot, GamepadComboInput gamepadComboInput) {
        this.robot = robot;
        this.gamepadComboInput = gamepadComboInput;
    }

    public void Init() {
        fieldCentric = false;
        resetDriveSlew();
    }


    @SuppressLint("DefaultLocale")
    public void DriveLoop() {
        // Toggle control mode
        boolean startNow = gamepadComboInput.getDriverStrSinglePressed() || gamepadComboInput.getOperatorStrSinglePressed();
        // START toggles field-centric mode.
        if (startNow && !startPressed ) {
            fieldCentric = !fieldCentric;
            startPressed = true;
        } else if (!startNow) {
            startPressed = false;
        }

        powerFactor = calculatePowerFactor();
        double drive = 0.0;
        double strafe = 0.0;
        double rotate = 0.0;

        // Gamepad 1 has priority over gamepad 2
        if(gamepadComboInput.getDriverHasDriveInput()) {
            drive = -gamepadComboInput.getDriverRightStickY();
            strafe = gamepadComboInput.getDriverRightStickX();
            rotate = gamepadComboInput.getDriverLeftStickX()*rotate_Slowness;
        } else if(gamepadComboInput.getOperatorHasDriveInput()) {
            drive = -gamepadComboInput.getOperatorRightStickY();
            strafe = gamepadComboInput.getOperatorRightStickX();
            rotate = gamepadComboInput.getOperatorLeftStickX()*rotate_Slowness;
        }

        if (fieldCentric) {
            double headingRadians = Math.toRadians(getRobotHeadingDegrees());
            double tempDrive = drive * Math.cos(headingRadians) - strafe * Math.sin(headingRadians);
            double tempStrafe = drive * Math.sin(headingRadians) + strafe * Math.cos(headingRadians);

            drive = tempDrive;
            strafe = tempStrafe;
        }

        setMecanumDrivePower(drive, strafe, rotate, powerFactor);
    }// end of driveloop


    private double calculatePowerFactor() {
        double trigger1 = gamepadComboInput.getDriverLeftTrigger();
        double trigger2 = gamepadComboInput.getOperatorLeftTrigger();
        double trigger = Math.max(trigger1, trigger2);

        if (trigger > 0.4) {
            return Range.clip(RobotActionConfig.powerFactor * (1.4 - trigger), 0.20, 1.0);
        }

        return RobotActionConfig.powerFactor;
    }

    private double getRobotHeadingDegrees() {
        double heading = robot.imu
                .getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        heading = AngleUnit.normalizeDegrees(heading);

        // Keep this sign if your IMU convention needs it.
        // If field-centric moves backward/sideways incorrectly, remove the negative sign.
        return -heading;
    }

    private double deadband(double input, double threshold) {
        if (Math.abs(input) < threshold) {
            return 0.0;
        }
        return input;
    }

    private void setMecanumDrivePower(double drive, double strafe, double rotate, double powerFactor) {

        double desiredFrontLeftPower = drive + strafe + rotate;
        double desiredFrontRightPower = drive - strafe - rotate;
        double desiredBackLeftPower = drive - strafe + rotate;
        double desiredBackRightPower = drive + strafe - rotate;

        double maxPower = Math.max(
                Math.max(Math.abs(desiredFrontLeftPower), Math.abs(desiredFrontRightPower)),
                Math.max(Math.abs(desiredBackLeftPower), Math.abs(desiredBackRightPower))
        );

        if (maxPower > 1.0) {
            desiredFrontLeftPower /= maxPower;
            desiredFrontRightPower /= maxPower;
            desiredBackLeftPower /= maxPower;
            desiredBackRightPower /= maxPower;
        }

        double frontLeftPower = frontLeftLimiter.calculate(desiredFrontLeftPower);
        double frontRightPower = frontRightLimiter.calculate(desiredFrontRightPower);
        double backLeftPower = backLeftLimiter.calculate(desiredBackLeftPower);
        double backRightPower = backRightLimiter.calculate(desiredBackRightPower);

        /** if use lerp method, here is the place to add it.

         *          lastFL = lerp(lastFL, frontLeft, RobotActionConfig.accel_Slowness, RobotActionConfig.decel_Slowness);
         *          lastFR = lerp(lastFR, frontRight, RobotActionConfig.accel_Slowness, RobotActionConfig.decel_Slowness);
         *          lastBL = lerp(lastBL, backLeft, RobotActionConfig.accel_Slowness, RobotActionConfig.decel_Slowness);
         *          lastBR = lerp(lastBR, backRight, RobotActionConfig.accel_Slowness, RobotActionConfig.decel_Slowness);
         */

        robot.frontLeftMotor.setPower(Range.clip(frontLeftPower * powerFactor, -1.0, 1.0));
        robot.frontRightMotor.setPower(Range.clip(frontRightPower * powerFactor, -1.0, 1.0));
        robot.backLeftMotor.setPower(Range.clip(backLeftPower * powerFactor, -1.0, 1.0));
        robot.backRightMotor.setPower(Range.clip(backRightPower * powerFactor, -1.0, 1.0));
    }

    public void resetDriveSlew() {
        frontLeftLimiter.reset(0.0);
        frontRightLimiter.reset(0.0);
        backLeftLimiter.reset(0.0);
        backRightLimiter.reset(0.0);
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    public double[] getVelocity() {
        double[] velocities = new double[4];
        velocities[0] = robot.frontLeftMotor.getVelocity();
        velocities[1] = robot.frontRightMotor.getVelocity();
        velocities[2] = robot.backLeftMotor.getVelocity();
        velocities[3] = robot.backRightMotor.getVelocity();
        return velocities;
    }

    public static class SlewRateLimiter {

        private final double accelmaxStep;
        private final double deaccelMaxStep;
        private double lastValue = 0.0;

        public SlewRateLimiter(double accelmaxStep, double deaccelMaxStep) {
            this.accelmaxStep = accelmaxStep;
            this.deaccelMaxStep = deaccelMaxStep;
        }

        public double calculate(double target) {
            double delta = target - lastValue;
            delta = Range.clip(delta, -deaccelMaxStep, accelmaxStep);
            lastValue += delta;
            return lastValue;
        }

        public void reset(double value) {
            lastValue = value;
        }
    }

    double lerp(double current, double target, double accel_Slowness, double decel_Slowness) {
        double alpha = (Math.abs(target) > Math.abs(current)) ? accel_Slowness : decel_Slowness;
        return current + alpha * (target - current);
    }
}
