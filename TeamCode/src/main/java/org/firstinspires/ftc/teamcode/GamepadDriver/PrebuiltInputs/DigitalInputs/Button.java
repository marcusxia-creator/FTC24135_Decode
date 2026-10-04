package org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.DigitalInputs;

import static org.apache.commons.math3.util.FastMath.ceil;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadInput;

import java.util.Arrays;
import java.util.function.Function;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class Button implements GamepadInput<Boolean> {
    private Stream<Gamepad> gamepads;
    private Function<Gamepad,Boolean> input;

    private Boolean[] inputs;
    private Boolean[] lastInputs;
    private final double debounceSeconds;
    private ElapsedTime debounceTimer;
    public boolean pressed;

    public Button(Gamepad[] gamepads, Function<Gamepad,Boolean> input, double debounceSeconds){
        this.gamepads=Arrays.stream(gamepads);
        this.input=input;

        lastInputs=getInputs();

        this.debounceSeconds=debounceSeconds;
        debounceTimer=new ElapsedTime((long)-ceil(debounceSeconds));
    }

    private Boolean[] getInputs(){
        return gamepads.map(gamepad->input.apply(gamepad)).toArray(Boolean[]::new);
    }

    @Override
    public void update() {
        lastInputs=inputs.clone();
        inputs=getInputs();
        pressed=checkPressed()&&debounceTimer.seconds()>=debounceSeconds;
        if(pressed){
            debounceTimer.reset();
        }
    }

    private boolean checkPressed(){
        return IntStream.range(0,(int)gamepads.count()).anyMatch(i->!lastInputs[i]&&inputs[i]);
    }

    @Override
    public Boolean get() {
        return pressed;
    }
}