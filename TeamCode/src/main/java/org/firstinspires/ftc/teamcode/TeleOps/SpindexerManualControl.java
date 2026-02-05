package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SpindexerManualControl {
    RobotHardware robot;
    SpindexerUpd spindexer;
    GamepadInput gamepadInput;
    ElapsedTime debounceTimer = new ElapsedTime();
    public SpindexerManualControl(RobotHardware robot, SpindexerUpd spindexer, GamepadInput gamepadInput){
        this.robot=robot;
        this.spindexer=spindexer;
        this.gamepadInput=gamepadInput;
    }
    public void loop() {
        /**
        if(gamepadInput.getDriverBackSinglePressed()) {
            spindexer.unJam();
        }
        */
        if(gamepadInput.getDriverLbSinglePressed()|| gamepadInput.getOperatorLbSinglePressed()){
            spindexer.RuntoPosition(spindexer.currentPos-1);
        }
        if((gamepadInput.getDriverRbSinglePressed() || gamepadInput.getOperatorRbSinglePressed())){
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
