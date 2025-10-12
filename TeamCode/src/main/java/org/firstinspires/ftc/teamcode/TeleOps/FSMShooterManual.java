package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
public class FSMShooterManual {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime();
    public enum SHOOTERSTATE{
        SHOOTER_START,
        SHOOTER_STOP
    }

    public FSMShooterManual (GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot){
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
    }
    public void Init(){
        robot.shooterMotor.setPower(0);
    }
    public void ShooterLoop(){
        if (gamepad_1.getButton(GamepadKeys.Button.Y)&&isButtonDebounced()){
            robot.shooterMotor.setPower(RobotActionConfig.shooterSpeed);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.X)&&isButtonDebounced()){
            robot.pushRampServo.setPosition(RobotActionConfig.rampPos);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.B) && isButtonDebounced()){
            robot.shooterMotor.setPower(0);
            robot.pushRampServo.setPosition(RobotActionConfig.rampResetPos);
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
