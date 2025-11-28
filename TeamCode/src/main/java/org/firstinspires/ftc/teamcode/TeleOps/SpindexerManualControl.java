package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SpindexerManualControl {
    RobotHardware robot;
    Spindexer spindexer;
    GamepadInput gamepadInput;
    GamepadManager gamepadManager;
    ElapsedTime debounceTimer = new ElapsedTime();
    public SpindexerManualControl(RobotHardware robot, Spindexer spindexer, GamepadManager gamepadManager, GamepadInput gamepadInput){
        this.robot=robot;
        this.spindexer=spindexer;
        this.gamepadManager=gamepadManager;
        this.gamepadInput=gamepadInput;
    }
    public void loop() {
        if(/**gamepadManager.Unjam.PressState*/gamepadInput.getDriverBackSinglePressed()) {
            spindexer.unJam();
        }
        if(/**gamepadManager.spinPrev.PressState*/gamepadInput.getDriverLbSinglePressed()|| gamepadInput.getOperatorLbSinglePressed()){
            spindexer.runToSlot(spindexer.currentSlot-1);
        }
        if(/**gamepadManager.spinNext.PressState*/(gamepadInput.getDriverRbSinglePressed() || gamepadInput.getOperatorRbSinglePressed())){
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
