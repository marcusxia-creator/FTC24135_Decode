package org.firstinspires.ftc.teamcode.TeleOps.Test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name= "Was_Just_Pressed_TEST", group = "org/firstinspires/ftc/teamcode/OpMode")
public class WasJustPressed extends OpMode {

    private static GamepadEx gamepadCo1;
    private static GamepadEx gamepadCo2;

    @Override
    public void init () {
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    @Override
    public void loop () {
        if (gamepadCo1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            telemetry.addLine("Left Bumper was just pressed");
        }
        if (gamepadCo1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            telemetry.addLine("Right Bumper was just pressed");
        }
        if (gamepadCo1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            telemetry.addLine("Left bumper was pressed");
        }

        telemetry.update();
    }
}
