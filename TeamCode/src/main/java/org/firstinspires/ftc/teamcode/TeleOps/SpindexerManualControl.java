package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SpindexerManualControl {
    RobotHardware robot;
    Spindexer spindexer;
    GamepadEx gamepad_1;
    GamepadManager gamepadManager;
    ElapsedTime debounceTimer = new ElapsedTime();
    public SpindexerManualControl(RobotHardware robot, Spindexer spindexer, GamepadManager gamepadManager, GamepadEx gamepad_1){
        this.robot=robot;
        this.spindexer=spindexer;
        this.gamepadManager=gamepadManager;
        this.gamepad_1 = gamepad_1;
    }
    public void loop() {
        if(/**gamepadManager.Unjam.PressState*/(!gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) &&
                gamepad_1.getButton(GamepadKeys.Button.BACK) &&
                isButtonDebounced())){
            spindexer.unJam();
        }
        if(/**gamepadManager.spinPrev.PressState*/(gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) &&
                !gamepad_1.getButton(GamepadKeys.Button.BACK) &&
                isButtonDebounced())){
            spindexer.runToSlot(spindexer.currentSlot-1);
        }
        if(/**gamepadManager.spinNext.PressState*/(gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER) &&
                !gamepad_1.getButton(GamepadKeys.Button.BACK) &&
                isButtonDebounced())){
            spindexer.runToSlot(spindexer.currentSlot+1);
        }

        if(gamepadManager.reDetc.PressState){
            spindexer.writeToCurrent(robot.colorSensor, robot.distanceSensor);
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
