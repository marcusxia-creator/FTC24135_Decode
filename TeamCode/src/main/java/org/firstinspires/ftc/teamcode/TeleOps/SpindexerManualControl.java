package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SpindexerManualControl {
    RobotHardware robot;
    SpindexerUpd spindexer;
    GamepadComboInput gamepadComboInput;
    ElapsedTime debounceTimer = new ElapsedTime();
    public SpindexerManualControl(RobotHardware robot, SpindexerUpd spindexer, GamepadComboInput gamepadComboInput){
        this.robot=robot;
        this.spindexer=spindexer;
        this.gamepadComboInput = gamepadComboInput;
    }
    public void loop() {
        /**
        if(gamepadComboInput.getDriverBackSinglePressed()) {
            spindexer.unJam();
        }
        */
        if(gamepadComboInput.getDriverLbSinglePressed()|| gamepadComboInput.getOperatorLbSinglePressed()){
            spindexer.RuntoPosition(spindexer.currentPos-1);
        }
        if((gamepadComboInput.getDriverRbSinglePressed() || gamepadComboInput.getOperatorRbSinglePressed())){
            spindexer.RuntoPosition(spindexer.currentPos+1);
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
