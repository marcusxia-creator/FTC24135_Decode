package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;

public class FSMShooter {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime rampTimer = new ElapsedTime();
    private SHOOTERSTATE shooterState;
    private int slotNumber = 1;


    boolean[] isFilled = {false, false, false};


    public enum SHOOTERSTATE {
        SHOOTER_START,
        RAMP_UP,
        COLOUR_SENSOR_CHECK,
        SPINDEXER_ROTATE,
        SHOOTER_STOP
    }

    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
    }

    public void Init() {
        robot.shooterMotor.setPower(0);
        robot.pushRampServo.setPosition(RobotActionConfig.rampDownPos);
        shooterState = SHOOTERSTATE.SHOOTER_START;
    }

    public void ShooterLoop() {
        switch (shooterState) {
            case SHOOTER_START:
                if (gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced()) {
                    robot.shooterMotor.setPower(RobotActionConfig.shooterSpeed);
                }
                if (isFlywheelAtSpeed(RobotActionConfig.shooterSpeed)) {
                    shooterState = SHOOTERSTATE.RAMP_UP;
                }
                break;
            case RAMP_UP:
                shootTimer.reset();
                if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()) {
                    robot.leftGateServo.setPosition(RobotActionConfig.gateUp);
                    robot.leftGateServo.setPosition(RobotActionConfig.gateUp);
                    robot.pushRampServo.setPosition(RobotActionConfig.rampUpPos);

                }
                if (shootTimer.seconds() > 1.0) {
                    robot.pushRampServo.setPosition(RobotActionConfig.rampDownPos);
                    shooterState = SHOOTERSTATE.COLOUR_SENSOR_CHECK;
                }
                break;
            case COLOUR_SENSOR_CHECK:
                double distance = robot.distanceSensor.getDistance(DistanceUnit.CM);
                if (distance < 10) {
                    shooterState = SHOOTERSTATE.SPINDEXER_ROTATE;
                } else {
                    shooterState = SHOOTERSTATE.RAMP_UP;
                }
                break;
            case SPINDEXER_ROTATE:
                if (slotNumber == 1) {
                    robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);
                    slotNumber = 2;
                    shooterState = SHOOTERSTATE.RAMP_UP;
                }
                else if (slotNumber == 2) {
                    robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot2);
                    slotNumber = 3;
                    shooterState = SHOOTERSTATE.RAMP_UP;
                }
                else if (slotNumber == 3) {
                    robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot3);
                    slotNumber = 1;
                    shooterState = SHOOTERSTATE.SHOOTER_STOP;
                }
                break;
            case SHOOTER_STOP:
                robot.shooterMotor.setPower(0.0);
                shooterState = SHOOTERSTATE.SHOOTER_START;
                break;
        }
    }

    private boolean isFlywheelAtSpeed(double targetSpeed) {
        double currentSpeed = robot.shooterMotor.getVelocity();
        return currentSpeed - targetSpeed > 1.0;
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

}

