package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GamepadManager {
    private ElapsedTime debounceTimer = new ElapsedTime();

    public GamepadEx gamepad_1;
    public GamepadEx gamepad_2;

    public enum ControlType{
                        //Pressed ↓-↓  ↓-↓
        Hold,               //____‾‾‾__‾‾‾__
        Press,              //____^____^____
        Toggle              //____‾‾‾‾‾_____
    }

    //Buttons
    public Control IntakeRun;
    public Control IntakeReverse;
    public Control Flywheel;
    public Control Launch;

    public GamepadManager(Gamepad gamepad1, Gamepad gamepad2){
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        IntakeRun = new Control(GamepadKeys.Button.DPAD_RIGHT, false);
        IntakeReverse = new Control(GamepadKeys.Button.DPAD_LEFT, false);
        Flywheel = new Control(GamepadKeys.Button.X, false);
        Launch = new Control(GamepadKeys.Button.Y, false);
    }

    public void loop(){
        IntakeRun.Update();
        IntakeReverse.Update();
        Flywheel.Update();
        Launch.Update();
    }

    public class Control{
        GamepadKeys.Button button;

        public Boolean HoldState;
        public Boolean prevHoldState;

        public Boolean PressState;

        public Boolean ToggleState;

        public Control(GamepadKeys.Button button, Boolean toggleStartState){
            this.button = button;
            ToggleState = toggleStartState;
            prevHoldState = gamepad_1.getButton(button)||gamepad_2.getButton(button) && isButtonDebounced();
        }

        public void Update(){
            HoldState = gamepad_1.getButton(button)||gamepad_2.getButton(button) && isButtonDebounced();
            PressState=!prevHoldState&&HoldState;
            if(PressState){
                ToggleState=!ToggleState;
            }
            prevHoldState=HoldState;
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