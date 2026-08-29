package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadDriver {
    Gamepad gamepad1;
    Gamepad gamepad2;

    public GamepadDriver(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1=gamepad1;
        this.gamepad2=gamepad2;
    }

    static public boolean joystickActive(Gamepad gamepad){
        double threshold=0.05;
        return Math.max(Math.max(Math.abs(gamepad.left_stick_x),Math.abs(gamepad.left_stick_y)),Math.max(Math.abs(gamepad.right_stick_x),Math.abs(gamepad.right_stick_y)))>=threshold;
    }

    private Gamepad activeGamepad(){
        return joystickActive(gamepad1)||!joystickActive(gamepad2)?gamepad1:gamepad2;
    }

    public double driveX(){
        return -activeGamepad().right_stick_x*0.5;
    }

    public double driveY(){
        return -activeGamepad().right_stick_y*0.5;
    }

    public double driveRot(){
        return activeGamepad().left_stick_x*0.5;
    }

    public boolean intakeStart(){
        return gamepad1.dpad_left||gamepad2.dpad_left;
    }

    public boolean cancel(){
        return gamepad1.b||gamepad2.b;
    }
}
