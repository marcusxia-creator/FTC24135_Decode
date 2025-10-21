
package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

public class IntakeBall {

    public enum INTAKEBALLSTATE{
        INTAKE_START,
        INTAKE_SWEEPING,
        INTAKE_DETECTED,
        INTAKE_INDEXING,
        INTAKE_FULL
    }

    private RobotHardware robot;
    private ColorDetection colorDetection;
    private ElapsedTime timer = new ElapsedTime();
    private INTAKEBALLSTATE state = INTAKEBALLSTATE.INTAKE_START;

    private List<Ball> balls = new ArrayList<>();
    private double[] slotAngles = {0.0, 120.0 / 180.0, 240.0 / 180.0};
    private int currentSlot = 0;

    private boolean colorDetected = false;
    private String detectedColor = "Unknown";

    // --- Constructor ---
    public IntakeBall(RobotHardware robot) {
        this.robot = robot;
        this.colorDetection = new ColorDetection(robot);
        this.robot.intakeIndexServo.setPosition(slotAngles[0]);
        timer.reset();
    }

    // --- FSM Update Loop ---
    public void update() {
        switch (state) {
            case INTAKE_START:
                balls.clear();
                currentSlot = 0;
                robot.intakeIndexServo.setPosition(slotAngles[0]);
                robot.intakeMotor.setPower(0.6);
                state = INTAKEBALLSTATE.INTAKE_SWEEPING;
                break;

            case INTAKE_SWEEPING:
                detectedColor = colorDetection.detectColorHue();

                if (!detectedColor.equals("Unknown")) {
                    colorDetected = true;
                    robot.intakeMotor.setPower(0.0);
                    timer.reset();
                    state = INTAKEBALLSTATE.INTAKE_DETECTED;
                }
                break;

            case INTAKE_DETECTED:
                if (colorDetected) {
                    Ball newBall = new Ball(detectedColor, currentSlot, slotAngles[currentSlot], false);
                    balls.add(newBall);
                    currentSlot++;

                    if (currentSlot < slotAngles.length) {
                        robot.intakeIndexServo.setPosition(slotAngles[currentSlot]);
                        timer.reset();
                        state = INTAKEBALLSTATE.INTAKE_INDEXING;
                    } else {
                        robot.intakeMotor.setPower(0.0);
                        state = INTAKEBALLSTATE.INTAKE_FULL;
                    }
                }
                break;

            case INTAKE_INDEXING:
                if (timer.seconds() > 0.5) {
                    colorDetected = false;
                    detectedColor = "Unknown";
                    robot.intakeMotor.setPower(0.6);
                    state = INTAKEBALLSTATE.INTAKE_SWEEPING;
                }
                break;

            case INTAKE_FULL:
                robot.intakeMotor.setPower(0.0);
                break;
        }
    }

    // --- Getters ---
    public INTAKEBALLSTATE getState() { return state; }
    public boolean isFull() { return state == INTAKEBALLSTATE.INTAKE_FULL; }
    public List<Ball> getBalls() { return balls; }
    public int getCurrentSlot() { return currentSlot; }
}
