
package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Runtime.getRuntime;
import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

public class IntakeBall {

       public enum INTAKEBALLSTATE{
        INTAKE_READY,
        INTAKE_SWEEPING,
        INTAKE_DETECTED,
        INTAKE_INDEXING,
        INTAKE_FULL,
        INTAKE_INDEXING_RETRY
    }
    private RobotHardware robot;
    private ColorDetection colorDetection;
    private final GamepadEx gamepad1;
    private final double[] slotAngles;
    private final List<Ball> balls;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime();

    // Example: gobilda 1150= 145.1 ticks/rev, 6000=28, GoBilda 5202/5203 = 537.7 ticks/rev.for 312rpm
    public static double SHOOTER_TICKS_PER_REV = 28;   // change to your motor
    public static double INTAKE_TICKS_PER_REV = 145.1;   // change to your motor
    public static double SHOOTER_RPM_CONVERSION = 60.0 / SHOOTER_TICKS_PER_REV;
    public static double INTAKE_RPM_CONVERSION = 60.0 / INTAKE_TICKS_PER_REV;

    private double intake_rpm;

    private INTAKEBALLSTATE state = INTAKEBALLSTATE.INTAKE_READY;

    private int currentSlot = 0;
    private int nextSlot;
    private int previousSlot = 0;

    private boolean colorDetected = false;
    private String detectedColor = "Unknown";

    // --- Constructor ---
    public IntakeBall(RobotHardware robot, GamepadEx gamepad, List<Ball> balls,double[] slotAngles) {
        this.robot = robot;
        this.gamepad1 = gamepad;
        this.colorDetection = new ColorDetection(robot);
        this.slotAngles = slotAngles;
        this.balls = balls;
        // --- NEW: Initialize the list with empty ball objects ---
        // This ensures the list always represents the 3 physical slots.
        for (int i = 0; i < this.slotAngles.length; i++) {
            // Add a "placeholder" ball for each slot, marked as not having a ball.
            this.balls.add(new Ball("Empty", i, this.slotAngles[i], false));
        }
        // --------------------------------------------------------
        this.robot.spindexerServo.setPosition(slotAngles[0]);
        timer.reset();
    }

    // --- FSM Update Loop ---
    public void IntkaeBallUpdate() {
        /// === Read velocity in ticks/sec and convert to RPM ===
        double intake_ticksPerSec = robot.intakeMotor.getVelocity();
        intake_rpm = intake_ticksPerSec * INTAKE_RPM_CONVERSION;

        /// * check on jammed or not /
        boolean jammed = isJammed();

       // Reverse slot position to previous one
       if (gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
                    // RETURN ball into the PREVIOUS slot
                    state = INTAKEBALLSTATE.INTAKE_INDEXING_RETRY;
                    timer.reset();
                }
       // FSM STATES
        switch (state) {
            case INTAKE_READY:
                currentSlot = findEmptySlot();
                if (currentSlot == -1) { // Should not happen after resetting, but a good safeguard.
                    currentSlot = 0;
                }
                robot.spindexerServo.setPosition(currentSlot);
                robot.leftGateServo.setPosition(RobotActionConfig.gateUp);
                robot.rightGateServo.setPosition(RobotActionConfig.gateUp);
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
                    robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
                    robot.rightGateServo.setPosition(RobotActionConfig.gateDown);

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
                    Ball currentSlotBall = balls.get(currentSlot);
                    currentSlotBall.hasBall = true;
                    currentSlotBall.ballColor = detectedColor;
                    stopIntake();
                    nextSlot = findEmptySlot();

                    if (!areAllSlotsFull()) {
                        ///if slot are not full, rotate slot
                        previousSlot = currentSlot;
                        currentSlot = nextSlot;
                        timer.reset();
                        state = INTAKEBALLSTATE.INTAKE_INDEXING;
                    } else {
                        state = INTAKEBALLSTATE.INTAKE_FULL;
                    }
                }
                break;

            case INTAKE_INDEXING:
                /// waiti spindexer rotating
                if (timer.seconds() > 0.25) {
                    robot.spindexerServo.setPosition(slotAngles[nextSlot]);
                    colorDetected = false;
                    detectedColor = "Unknown";
                    robot.intakeMotor.setPower(0.6);
                }
                if (timer.seconds()>0.5){
                    state = INTAKEBALLSTATE.INTAKE_SWEEPING;
                    robot.leftGateServo.setPosition(RobotActionConfig.gateUp);
                    robot.rightGateServo.setPosition(RobotActionConfig.gateUp);
                    timer.reset();
                }
                break;

            case INTAKE_FULL:
                      stopIntake();
                break;

            case INTAKE_INDEXING_RETRY:
                      double currentTime = getRunTime();
                      robot.spindexerServo.setPosition(slotAngles[previousSlot]);
                      if (getRuntime()- currentTime > 0.5) {
                            inTakeBallState = INTAKEBALLSTATE.INTAKE_READY
                            }
                     timer.reset();
              break;
        }
    }

    // --- Getters ---
    public INTAKEBALLSTATE getState() { return state; }
    public void setState(INTAKEBALLSTATE state) { this.state = state; }
    public boolean isReady() { return state == INTAKEBALLSTATE.INTAKE_READY; }
    public INTAKEBALLSTATE state() { return state;}
    public boolean isFull() { return state == INTAKEBALLSTATE.INTAKE_FULL; }
    public List<Ball> getBalls() { return balls; }
    public int getCurrentSlot() { return currentSlot; }
    public void resetSpindexerSlot(){ currentSlot = 0; robot.spindexerServo.setPosition(0); balls.clear();}

    ///  Intake Stop Helper
    public void stopIntake() {
        robot.intakeMotor.setPower(0.0);
    }

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
        return getNumberOfBalls() == 3;
    }

    // Add this helper method inside your IntakeBall.java class

    /**
     * Finds the index of the first available (empty) slot in the spindexer.
     *
     * @return The index of the first empty slot, or -1 if all slots are full.
     */
    public int findEmptySlot() {
        // Iterate through each ball object, which represents a physical slot.
        for (Ball b : balls) {
            // Check if this slot is marked as empty.
            if (!b.hasBall) {
                // If it's empty, return its position index.
                return b.getSlotPosition();
            }
        }
        // If the loop finishes, it means no ball had hasBall = false, so all slots are full.
        return -1;
    }

    public int getNumberOfBalls() {
        int ballCount = 0;
        // Loop through every ball in the shared list
        for (Ball b : balls) {
            // If the ball's hasBall flag is true, increment the counter
            if (b.hasBall) {
                ballCount++;
            }
        }
        return ballCount;
    }
}
