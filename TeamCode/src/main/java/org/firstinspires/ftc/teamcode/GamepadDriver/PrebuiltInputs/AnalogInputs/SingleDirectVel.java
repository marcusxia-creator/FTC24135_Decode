package org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.AnalogInputs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadInput;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;

import java.util.function.Function;

public class SingleDirectVel implements GamepadInput<Velocity> {
    static double deadzone=0.02;

    Scalar velFactor;
    Scalar angFactor;

    Gamepad gamepad;

    Function<Gamepad,Double> drive;
    Function<Gamepad,Double> strafe;
    Function<Gamepad,Double> rot;

    public SingleDirectVel(Gamepad gamepad, Function<Gamepad,Double> drive, Function<Gamepad,Double> strafe, Function<Gamepad,Double> rot, Scalar velFactor, Scalar angFactor){
        this.velFactor=velFactor;
        this.angFactor=angFactor;

        this.gamepad=gamepad;

        this.drive=drive;
        this.strafe=strafe;
        this.rot=rot;
    }

    DimlessVector getLinInput(){
        return new DimlessVector(drive.apply(gamepad),strafe.apply(gamepad));
    }

    double getRotInput(){
        return rot.apply(gamepad);
    }

    @Override
    public Velocity get() {
        return new Velocity(getLinInput().multi(velFactor),angFactor.multiply(getRotInput()));
    }

    public boolean active(){
        return getLinInput().mag()>=deadzone&&getRotInput()>=deadzone;
    }
}
