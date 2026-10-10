package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.metersPerSecond;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.metersPerSecondSquared;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.radiansPerSecond;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.radiansPerSecondSquared;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadDriver;
import org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.AnalogInputs.*;
import org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.DigitalInputs.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;

@Config
public class GamepadSubsystem extends GamepadDriver {
    public static Scalar velFactor=new Scalar(7,metersPerSecond);
    public static Scalar angFactor=new Scalar(5,radiansPerSecond);

    //Lerping
    public static Scalar maxAccel=new Scalar(9.81,metersPerSecondSquared);
    public static Scalar maxAngAccel=new Scalar(5,radiansPerSecondSquared);

    public static double debounce=0.2;

    public SingleDirectVel joystick1;
    public SingleDirectVel joystick2;

    public LerpedVel driveInput;

    public Toggle intakeToggle;

    public Switch driveSelect;

    public GamepadSubsystem(Gamepad gamepad1, Gamepad gamepad2){
        Gamepad[] gamepads={gamepad1,gamepad2};

        joystick1=new SingleDirectVel(gamepad1,gamepad ->(double)gamepad.right_stick_y,gamepad ->(double)gamepad.right_stick_x,gamepad -> (double)gamepad.left_stick_x,velFactor,angFactor);
        joystick2=new SingleDirectVel(gamepad2,gamepad ->(double)gamepad.right_stick_y,gamepad ->(double)gamepad.right_stick_x,gamepad -> (double)gamepad.left_stick_x,velFactor,angFactor);

        driveInput=new LerpedVel(new DualDirectVel(joystick1,joystick2),maxAccel,maxAngAccel);

        intakeToggle=new Toggle(gamepads,gamepad -> gamepad.a,debounce,false);

        driveSelect=new Switch(gamepads,gamepad -> joystick1.active()||joystick2.active(),gamepad -> gamepad.left_trigger>=0.9&&gamepad.x);
    }
}
