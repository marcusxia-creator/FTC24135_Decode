
package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

public class IntakeBall {

       public enum INTAKEBALLSTATE{
        INTAKE_READY,
        INTAKE_SWEEPING,
        INTAKE_DETECTED,
        INTAKE_INDEXING,
        INTAKE_FULL,
        INTAKE_UNJAMMING
    }
    //=============SUBSYSTEMS=================================
    private RobotHardware robot;
    private final GamepadEx gamepad1;

    //============TIMER==================================
    private final ElapsedTime runtime = new ElapsedTime();  //run time
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime();
    // Example: gobilda 1150= 145.1 ticks/rev, 6000=28, GoBilda 5202/5203 = 537.7 ticks/rev.for 312rpm
    public static double INTAKE_TICKS_PER_REV = 145.1;   // change to your motor
    public static double INTAKE_RPM_CONVERSION = 60.0 / INTAKE_TICKS_PER_REV;
    //============INTAKE MOTOR==================================
    private double intake_rpm;
    //===========INTAKE STATE===================================
    private INTAKEBALLSTATE state = INTAKEBALLSTATE.INTAKE_READY;
    //===========SPINDEXER===================================
    private final double[] slotAngles;
    private List<Ball> balls = new ArrayList<>() ;
    private int currentSlot = 0;
    private int nextSlot;
    private int previousSlot = 0;
    //==========COLOR DETECTION================================
    private ColorDetection colorDetection;
    private boolean colorDetected = false;
    private String detectedColor = "Unknown";

    // --- Constructor ---
    public IntakeBall(RobotHardware robot, GamepadEx gamepad, List<Ball> balls, double[] slotAngles) {
        this.robot = robot;
        this.gamepad1 = gamepad;
        this.balls = balls;
        this.slotAngles = slotAngles;

        // --------------------------------------------------------
        this.robot.spindexerServo.setPosition(slotAngles[0]);
        this.colorDetection = new ColorDetection(robot);
        timer.reset();
        runtime.reset();
    }

    // --- FSM Update Loop ---
    public void IntakeBallUpdate() {
        /// === Read velocity in ticks/sec and convert to RPM ===
        double intake_ticksPerSec = robot.intakeMotor.getVelocity();
        intake_rpm = intake_ticksPerSec * INTAKE_RPM_CONVERSION;

        /// * check on jammed or not /
        boolean jammed = isJammed();

       // Reverse slot position to previous one
       if (gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
                    // RETURN ball into the PREVIOUS slot
                    state = INTAKEBALLSTATE.INTAKE_UNJAMMING;
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
                colorDetection.updateDetection();
                ///color detection - checking ball colors
                if (colorDetection.isColorStable()) {
                    colorDetected = true;
                    detectedColor = colorDetection.getStableColor();
                    balls.get(currentSlot).setHasBall(true);
                    balls.get(currentSlot).setBallColor(detectedColor);
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

            case INTAKE_UNJAMMING:
                      double currentTime = runtime.seconds();
                      robot.spindexerServo.setPosition(slotAngles[previousSlot]);
                      if (runtime.seconds()- currentTime > 0.35) {
                          currentSlot = previousSlot;
                          balls.get(previousSlot).setHasBall(false);
                            state = INTAKEBALLSTATE.INTAKE_SWEEPING;
                            }
                     timer.reset();
               break;
        }
    }

    /// --- Getters ---
    public INTAKEBALLSTATE getState() { return state; }
    public boolean isReady() { return state == INTAKEBALLSTATE.INTAKE_READY; }
    public boolean isFull() { return state == INTAKEBALLSTATE.INTAKE_FULL; }
    public void resetSpindexerSlot(){ currentSlot = 0; robot.spindexerServo.setPosition(0); balls.clear();}
    public List<Ball> getBalls() { return balls; }
    public String getDetectedColor() { return detectedColor; }
    public boolean isColorDetected() { return colorDetected; }

    /// --- Setter ---
    public void setState(INTAKEBALLSTATE state) { this.state = state; }
    public int getCurrentSlot() { return currentSlot; }

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

    ///helper to determine spindexer full or not.
    public boolean areAllSlotsFull() {
        return getNumberOfBalls() == 3;
    }

    // Add this helper method inside your IntakeBall.java class
    public Ball getBallBySlot(int slotNumber) {
        if (slotNumber < 0 || slotNumber >= balls.size()) return null;
        return balls.get(slotNumber);
    }

    public void setSlotBall(int slotNumber, String color) {
        Ball b = getBallBySlot(slotNumber);
        if (b != null) {
            b.setHasBall(true);
            b.setBallColor(color);
        }
    }

    public void setSlotEmpty(int slotNumber) {
        Ball b = getBallBySlot(slotNumber);
        if (b != null) {
            b.setHasBall(false);
            b.setBallColor("Empty");
        }
    }

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

    /// Button Debounce Helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
}
