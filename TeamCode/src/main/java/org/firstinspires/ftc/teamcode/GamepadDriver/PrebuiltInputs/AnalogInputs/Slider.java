package org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.AnalogInputs;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadInput;

import java.util.function.Function;

public class Slider implements GamepadInput<Double>{
    double position;

    final double step;
    final double lowerBound;
    final double upperBound;

    GamepadInput<Boolean>increment;
    GamepadInput<Boolean>decrement;

    public Slider(double step,double lowerBound,double upperBound,GamepadInput<Boolean>increment,GamepadInput<Boolean>decrement,double init){
        this.step=step;
        this.lowerBound=lowerBound;
        this.upperBound=upperBound;

        this.increment=increment;
        this.decrement=decrement;

        position=init;
    }

    @Override
    public void update() {
        increment.update();
        decrement.update();

        position+=increment.get()?step:0;
        position-=decrement.get()?step:0;

        position=Range.clip(position,lowerBound,upperBound);
    }

    @Override
    public Double get() {
        return position;
    }
}
