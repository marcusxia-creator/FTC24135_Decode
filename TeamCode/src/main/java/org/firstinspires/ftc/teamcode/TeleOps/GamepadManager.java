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
    public Control Green;
    public Control Purple;
    public Control Unjam;
    public Control spinPrev;
    public Control spinNext;
    public Control reDetc;
    public Control autoMotif;
    public Control ToggleShooterPower;

    public GamepadManager(Gamepad gamepad1, Gamepad gamepad2){
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        IntakeRun = new Control(GamepadKeys.Button.DPAD_LEFT, false);
        IntakeReverse = new Control(GamepadKeys.Button.DPAD_RIGHT, false);
        Flywheel = new Control(GamepadKeys.Button.X, false);
        Launch = new Control(GamepadKeys.Button.Y, false);
        Green = new Control(GamepadKeys.Button.A, false);
        Purple = new Control(GamepadKeys.Button.B, false);
        Unjam = new Control(GamepadKeys.Button.BACK, false);
        spinPrev = new Control(GamepadKeys.Button.LEFT_BUMPER, false);
        spinNext = new Control(GamepadKeys.Button.RIGHT_BUMPER, false);
        reDetc = new Control(GamepadKeys.Button.DPAD_UP, false);
        autoMotif = new Control(GamepadKeys.Button.START, false);
        //ToggleShooterPower = new Control(GamepadKeys.Button.LEFT_BUMPER && GamepadKeys.Button.BACK, false);
    }

    public void loop(){
        IntakeRun.Update();
        IntakeReverse.Update();
        Flywheel.Update();
        Green.Update();
        Purple.Update();
        Unjam.Update();
        spinPrev.Update();
        spinNext.Update();
        reDetc.Update();
        autoMotif.Update();
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