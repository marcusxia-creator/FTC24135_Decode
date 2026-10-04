package org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.AnalogInputs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadInput;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;

import java.util.function.Function;

public class DualDirectVel implements GamepadInput<Velocity> {
    SingleDirectVel drive1;
    SingleDirectVel drive2;

    public DualDirectVel(SingleDirectVel drive1, SingleDirectVel drive2){
        this.drive1=drive1;
        this.drive2=drive2;
    }

    public DualDirectVel(Gamepad gamepad1, Gamepad gamepad2, Function<Gamepad,Double> drive, Function<Gamepad,Double> strafe, Function<Gamepad,Double> rot, Scalar velFactor, Scalar angFactor){
        drive1=new SingleDirectVel(gamepad1,drive,strafe,rot,velFactor,angFactor);
        drive2=new SingleDirectVel(gamepad2,drive,strafe,rot,velFactor,angFactor);
    }

    @Override
    public Velocity get() {
        return drive1.active()||!drive2.active()?drive1.get():drive2.get();
    }
}
