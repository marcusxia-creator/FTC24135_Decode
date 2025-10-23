
package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

public class IntakeBall {

    private final GamepadEx gamepad1;

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

    private ElapsedTime jamTimer = new ElapsedTime();

    private ElapsedTime debounceTimer = new ElapsedTime();

    // Example: gobilda 1150= 145.1 ticks/rev, 6000=28, GoBilda 5202/5203 = 537.7 ticks/rev.for 312rpm
    public static double SHOOTER_TICKS_PER_REV = 28;   // change to your motor
    public static double INTAKE_TICKS_PER_REV = 145.1;   // change to your motor
    public static double SHOOTER_RPM_CONVERSION = 60.0 / SHOOTER_TICKS_PER_REV;
    public static double INTAKE_RPM_CONVERSION = 60.0 / INTAKE_TICKS_PER_REV;

    private double shooter_rpm;
    private double intake_rpm;

    private INTAKEBALLSTATE state = INTAKEBALLSTATE.INTAKE_START;

    private List<Ball> balls = new ArrayList<>();
    private double[] slotAngles = {0.0, 120.0 / 290.0, 240.0 / 290.0};
    private int currentSlot = 0;
    private int nextSlot;

    private boolean colorDetected = false;
    private String detectedColor = "Unknown";

    // --- Constructor ---
    public IntakeBall(RobotHardware robot, GamepadEx gamepad) {
        this.robot = robot;
        this.gamepad1 = gamepad;
        this.colorDetection = new ColorDetection(robot);
        this.robot.spindexerServo.setPosition(slotAngles[0]);
        timer.reset();
    }

    // --- FSM Update Loop ---
    public void IntkaeBallUpdate() {
        // === Read velocity in ticks/sec and convert to RPM ===
        double intake_ticksPerSec = robot.intakeMotor.getVelocity();
        intake_rpm = intake_ticksPerSec * INTAKE_RPM_CONVERSION;

        boolean jammed = isJammed();

        switch (state) {
            case INTAKE_START:
                balls.clear();
                currentSlot = 0;
                robot.spindexerServo.setPosition(slotAngles[0]);

                if (gamepad1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
                    //start the intake motor
                    robot.intakeMotor.setPower(0.6);
                    // waiting ball into the slot
                    state = INTAKEBALLSTATE.INTAKE_SWEEPING;
                    timer.reset();
                }
                break;

            case INTAKE_SWEEPING:
                /// intaking balls
                /// detect if the ball is jammed
                if (jammed) {
                    robot.intakeMotor.setPower(-0.2);
                    try {
                        sleep(300);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    robot.intakeMotor.setPower(0.0);
                }
                /// check if the ball is present at slot
                if (colorDetection.isBallPresent()) {
                    colorDetection.startDetection(); // set color detection parameters to initial start.
                    timer.reset();
                    state = INTAKEBALLSTATE.INTAKE_DETECTED;
                }
                break;

            case INTAKE_DETECTED:
                ///color detection - checking ball colors
                colorDetection.updateDetection();

                if (colorDetection.isColorStable()) {
                    colorDetected = true;
                    detectedColor = colorDetection.getStableColor();
                    Ball newBall = new Ball(detectedColor, currentSlot, slotAngles[currentSlot], true);
                    balls.add(newBall);
                    nextSlot = (currentSlot+1)%3;

                    if (!areAllSlotsFull()) {
                        ///if slot are not full, rotate slot
                        robot.spindexerServo.setPosition(slotAngles[nextSlot]);
                        currentSlot=nextSlot;
                        timer.reset();
                        state = INTAKEBALLSTATE.INTAKE_INDEXING;
                    } else {
                        state = INTAKEBALLSTATE.INTAKE_FULL;
                    }
                } else if (timer.seconds() > 0.8){
                    ///Timeout safer
                    state = INTAKEBALLSTATE.INTAKE_INDEXING;
                }
                break;

            case INTAKE_INDEXING:
                /// waiti spindexer rotating
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
    public void setState(INTAKEBALLSTATE state) { this.state = state; }
    public boolean isFull() { return state == INTAKEBALLSTATE.INTAKE_FULL; }
    public List<Ball> getBalls() { return balls; }
    public int getCurrentSlot() { return currentSlot; }
    public void resetSpindexerSlot(){ currentSlot = 0; robot.spindexerServo.setPosition(0); }

    /// Intake Jam Helper
    private boolean isJammed() {
        if (robot.intakeMotor.getPower() > 0.2 && intake_rpm < RobotActionConfig.intakeRPM_THRESHOLD) {
            if (jamTimer.seconds() > 0.3) return true; // jam confirmed for 0.3s
        } else {
            jamTimer.reset();
        }
        return false;
    }

    /// Button Debounce Helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
    ///helper to determine spindexer full or not.
    public boolean areAllSlotsFull() {
        int fullCount = 0;
        for (Ball b : balls) {
            if (b.hasBall) fullCount++;
        }
        return fullCount == 3;
    }
}
