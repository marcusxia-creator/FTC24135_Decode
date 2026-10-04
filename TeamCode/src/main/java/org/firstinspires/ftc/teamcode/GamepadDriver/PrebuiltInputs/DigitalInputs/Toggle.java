package org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.DigitalInputs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadInput;

import java.util.function.Function;

public class Toggle implements GamepadInput<Boolean> {
    Button button;
    Boolean state;

    public Toggle(Button button, boolean startingState){
        this.button=button;
        state=startingState;
    }

    public Toggle(Gamepad[] gamepads,Function<Gamepad,Boolean> input, double debounceSeconds,boolean startingState){
        this.button=new Button(gamepads,input,debounceSeconds);
        state=startingState;
    }



    @Override
    public void update() {
        button.update();
        if(button.get()){
            state=!state;
        }
    }

    @Override
    public Boolean get() {
        return state;
    }
}