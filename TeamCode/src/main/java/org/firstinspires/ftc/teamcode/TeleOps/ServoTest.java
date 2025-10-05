package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class ServoTest {

    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private RobotHardware robot;

    private double servoposition;
    private double servoposition2;
    private static final int delta_Position = 50;
    private int current_Position;

    private static final double speed = 1.0;

    private final ElapsedTime debounceTimer = new ElapsedTime();
    private static final double DEBOUNCE_THRESHOLD = 0.25;

    private static boolean leftButtonPressed = false;

    public ServoTest(RobotHardware robot, GamepadEx gamepad1, GamepadEx gamepad2) {
        this.gamepad_1 = gamepad1;
        this.gamepad_2 = gamepad2;
        this.robot = robot;
    }

    public void init() {

        servoposition = 0.0;
        servoposition2 = 0.0;
    }
}