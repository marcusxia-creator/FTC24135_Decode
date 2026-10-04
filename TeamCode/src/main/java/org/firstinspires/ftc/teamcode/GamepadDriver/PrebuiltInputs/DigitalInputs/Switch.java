package org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.DigitalInputs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadInput;

import java.util.Arrays;
import java.util.function.Function;
import java.util.stream.IntStream;

public class Switch implements GamepadInput<Integer>{
    Hold[] holds;
    int position;

    public Switch(Hold... holds){
        position=0;
        this.holds=holds;
    }

    public Switch(Gamepad[] gamepads, Function<Gamepad,Boolean>... inputs){
        position=0;
        this.holds=Arrays.stream(inputs).map(input->new Hold(gamepads,input)).toArray(Hold[]::new);
    }

    @Override
    public void update() {
        Arrays.stream(holds).forEach(Hold::update);
        if(Arrays.stream(holds).anyMatch(Hold::get)){
            position=IntStream.range(0,holds.length).filter(i->holds[i].get()).findFirst().orElse(position);
        }
    }

    @Override
    public Integer get() {
        return position;
    }
}
