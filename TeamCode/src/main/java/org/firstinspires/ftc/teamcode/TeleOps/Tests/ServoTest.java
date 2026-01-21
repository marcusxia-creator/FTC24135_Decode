package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Button Control", group = "Test")
public class ServoTest extends OpMode {

    private Servo testServo;

    private double servoPos = 0.5;        // Start at middle
    private static final double STEP = 0.25;

    // Button state tracking (debounce)
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void init() {
        testServo = hardwareMap.get(Servo.class, "Servo");
        testServo.setPosition(servoPos);

        telemetry.addLine("Servo initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===============================
        // Continuous rotation (direction)
        // ===============================
        if (gamepad1.left_bumper) {
            testServo.setDirection(Servo.Direction.FORWARD);   // normal direction
        }
        else if (gamepad1.right_bumper) {
            testServo.setDirection(Servo.Direction.REVERSE);   // normal direction
        }

        // ===============================
        // Incremental step control
        // ===============================
        if (gamepad1.dpad_left && !lastDpadLeft) {
            servoPos += STEP;
        }

        if (gamepad1.dpad_right && !lastDpadRight) {
            servoPos -= STEP;
        }

        // Save last states
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        // ===============================
        // Clamp servo range
        // ===============================
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));

        // Apply position
        testServo.setPosition(servoPos);

        // ===============================
        // Telemetry
        // ===============================
        telemetry.addData("Servo Position", "%.2f", servoPos);
        telemetry.addData("LB", gamepad1.left_bumper);
        telemetry.addData("Servo Direction", testServo.getDirection());
        telemetry.addData("RB", gamepad1.right_bumper);
        telemetry.update();
    }
}

