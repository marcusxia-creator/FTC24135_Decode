package org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.DigitalInputs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadInput;

import java.util.Arrays;
import java.util.function.Function;

public class Hold implements GamepadInput<Boolean> {
    final Gamepad[] gamepads;
    Function<Gamepad,Boolean> input;
    public Hold(Gamepad[] gamepads, Function<Gamepad,Boolean> input){
        this.gamepads=gamepads;
        this.input=input;
    }

    @Override
    public Boolean get() {
        return Arrays.stream(gamepads).anyMatch(gamepad -> input.apply(gamepad));
    }
}