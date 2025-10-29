package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FSMShooter {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime rampTimer = new ElapsedTime();
    private SHOOTERSTATE shooterState;
    public RAMPSTATE rampstate = RAMPSTATE.DOWN;
    private int counter = 2;


   // double [] isFilled = {false, false, false};
    double [] slotAngle = {0.02, 0.46, 0.90};

    public enum SHOOTERSTATE {
        FLYWHEEL_START,
        RAMP_UP,
        DISTANCE_SENSOR_CHECK,
        SPINDEXER_ROTATE,
    }
    public enum RAMPSTATE{
        UP,
        DOWN
    }

    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
    }

    public void Init() {
        counter = 2;
        robot.shooterMotor.setPower(0);
        robot.pushRampServo.setPosition(RobotActionConfig.rampDownPos);
        shooterState = SHOOTERSTATE.FLYWHEEL_START;
    }

    public void ShooterLoop() {
        switch (shooterState) {
            case FLYWHEEL_START:
                if (gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced()) {
                    robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.shooterMotor.setPower(RobotActionConfig.shooterSpeed);
                    if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()) {
                        shooterState = SHOOTERSTATE.RAMP_UP;
                    }
                    shootTimer.reset();
                }
                break;
            case RAMP_UP:
                toggleRamp();
                updateServoState();
                shootTimer.reset();
                break;
            case DISTANCE_SENSOR_CHECK:
                if (shootTimer.seconds() > 0.5) {
                    toggleRamp();
                    updateServoState();
                    double distance = robot.distanceSensor.getDistance(DistanceUnit.MM);
                    if (distance < 100) {
                        shooterState = SHOOTERSTATE.RAMP_UP;
                        shootTimer.reset();
                    } else {
                        robot.pushRampServo.setPosition(RobotActionConfig.rampDownPos);
                        shooterState = SHOOTERSTATE.SPINDEXER_ROTATE;
                        shootTimer.reset();
                    }
                }
                break;
            case SPINDEXER_ROTATE:
                if (counter > 0) {
                    counter = counter - 1;
                    robot.spindexerServo.setPosition(slotAngle[counter]);
                    if (shootTimer.seconds() > 0.2) {
                        shooterState = SHOOTERSTATE.RAMP_UP;
                    }
                } else if (counter == 0) {
                    shooterState = SHOOTERSTATE.FLYWHEEL_START;
                }
                shootTimer.reset();
                break;
            default:
                shooterState = SHOOTERSTATE.FLYWHEEL_START;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.A) && isButtonDebounced()){
            robot.pushRampServo.setPosition(RobotActionConfig.rampDownPos);
            robot.shooterMotor.setPower(0);
            shooterState = SHOOTERSTATE.FLYWHEEL_START;
        }
    }
    private void toggleRamp (){
        if (rampstate == RAMPSTATE.UP){
            rampstate = RAMPSTATE.DOWN;
        }else{
            rampstate = RAMPSTATE.UP;
        }
    }
    private void updateServoState (){
        if (rampstate != RAMPSTATE.UP){
            robot.pushRampServo.setPosition(RobotActionConfig.rampDownPos);
        } else{
            robot.pushRampServo.setPosition(RobotActionConfig.rampUpPos);
        }
    }


    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

}

