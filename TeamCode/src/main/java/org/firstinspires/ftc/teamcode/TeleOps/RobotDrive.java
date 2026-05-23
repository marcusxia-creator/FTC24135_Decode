package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * --------------------------------------------------------
 *                    DRIVER CONTROLS
 * --------------------------------------------------------
 *
 * GAMEPAD PRIORITY:
 * - Gamepad 1 overrides Gamepad 2 for drivetrain control.
 *
 * --------------------------------------------------------
 * DRIVETRAIN CONTROLS
 * --------------------------------------------------------
 *
 * RIGHT STICK Y
 * - Forward / backward drive
 *
 * RIGHT STICK X
 * - Strafe left / right
 *
 * LEFT STICK X
 * - Robot rotation / turning
 *
 * --------------------------------------------------------
 * DRIVE FEATURES
 * --------------------------------------------------------
 *
 * LEFT TRIGGER
 * - Precision / slow driving mode
 * - Gradually reduces drivetrain power
 * - Useful for alignment and scoring
 *
 * START BUTTON
 * - Toggle Field-Centric / Robot-Centric drive mode
 *
 *      Field-Centric:
 *      - Joystick direction is relative to field
 *      - Forward always means field-forward
 *
 *      Robot-Centric:
 *      - Joystick direction is relative to robot
 *      - Forward means robot-forward
 *
 * LEFT_BUMPER + START
 * - Reserved for command combinations
 * - Prevents accidental field-centric toggling
 *
 * --------------------------------------------------------
 * DRIVE SYSTEM FEATURES
 * --------------------------------------------------------
 *
 * - Mecanum drive normalization
 * - Independent slew-rate limiting for all 4 motors
 * - Deadband filtering to remove joystick noise
 * - Optional field-centric transformation using IMU heading
 * - Smooth acceleration/deceleration control
 * - Power scaling for precision movement
 *
 * --------------------------------------------------------
 * CONTROL PIPELINE
 * --------------------------------------------------------
 *
 * Joystick Input
 *      ->
 * Deadband Filtering
 *      ->
 * Field-Centric Transform (optional)
 *      ->
 * Mecanum Mixing
 *      ->
 * Power Normalization
 *      ->
 * Slew Rate Limiting
 *      ->
 * Motor Power Output
 *
 * --------------------------------------------------------
 */


public class RobotDrive {

    private final GamepadComboInput gamepadComboInput;
    private final RobotHardware robot;

    private boolean startPressed = false;
    private boolean fieldCentric = false;

    private double powerFactor;
    private final double accel_Slowness = 0.45;
    private final double decel_Slowness = 0.25;

    /**
     * private double lastFL = 0;
     * private double lastFR = 0;
     * private double lastBL = 0;
     * private double lastBR = 0;
    */
    private final SlewRateLimiter frontLeftLimiter = new SlewRateLimiter(0.2,0.45);
    private final SlewRateLimiter frontRightLimiter = new SlewRateLimiter(0.2,0.45);
    private final SlewRateLimiter backLeftLimiter = new SlewRateLimiter(0.2,0.45);
    private final SlewRateLimiter backRightLimiter = new SlewRateLimiter(0.2,0.45);

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
    }

    /**
     if (hasDriveInput(gamepad_1)) {
     drive = deadband(-gamepad_1.getRightY(), 0.10);
     strafe = deadband(gamepad_1.getRightX(), 0.10);
     rotate = deadband(gamepad_1.getLeftX(), 0.10) * rotateSlowness;
     } else if (hasDriveInput(gamepad_2)) {
     drive = deadband(-gamepad_2.getRightY(), 0.10);
     strafe = deadband(gamepad_2.getRightX(), 0.10);
     rotate = deadband(gamepad_2.getLeftX(), 0.10) * rotateSlowness;
     }
     private boolean hasDriveInput(GamepadEx gamepad) {
     * return Math.abs(gamepad.getRightY()) > 0.10
     *      || Math.abs(gamepad.getRightX()) > 0.10
     *      || Math.abs(gamepad.getLeftX()) > 0.10;
     *      }
     */

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
