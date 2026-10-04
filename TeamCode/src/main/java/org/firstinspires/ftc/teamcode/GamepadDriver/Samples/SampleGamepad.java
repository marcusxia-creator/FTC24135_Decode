package org.firstinspires.ftc.teamcode.GamepadDriver.Samples;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadDriver;
import org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.AnalogInputs.*;
import org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.DigitalInputs.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;

public class SampleGamepad extends GamepadDriver {
    public static double debounce=0.1;

    Gamepad gamepad1;
    Gamepad gamepad2;

    Gamepad[] gamepads;

    public LerpedVel drive;
    public Toggle intakeToggle;
    public Slider intakePower;

    public SampleGamepad(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1=gamepad1;
        this.gamepad2=gamepad2;
        gamepads= new Gamepad[]{gamepad1, gamepad2};

        drive=new LerpedVel(gamepad1,gamepad2,(gamepad) -> (double)gamepad.right_stick_y,(gamepad) -> (double)gamepad.right_stick_x,(gamepad) -> (double)gamepad.left_stick_x,new Scalar(6,metersPerSecond),new Scalar(6,radiansPerSecond),new Scalar(5,metersPerSecondSquared),new Scalar(5,radiansPerSecondSquared));

        intakeToggle=new Toggle(gamepads,gamepad -> gamepad.a,debounce,false);

        intakePower=new Slider(0.1,0,1,new Button(gamepads,gamepad -> gamepad.dpad_up,debounce),new Button(gamepads,gamepad -> gamepad.dpad_down,debounce),0.5);
    }
}
