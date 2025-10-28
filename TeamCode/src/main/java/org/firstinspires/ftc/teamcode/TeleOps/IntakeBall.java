package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class IntakeBall {

    public enum INTAKEBALLSTATE {
        INTAKE_READY,
        INTAKE_SWEEPING,
        INTAKE_DETECTED,
        INTAKE_INDEXING,
        INTAKE_FULL,
        INTAKE_UNJAMMING
    }

    //============= SUBSYSTEMS ===============================
    private RobotHardware robot;
    private final GamepadEx gamepad1;

    //============= TIMERS ===================================
    private final ElapsedTime runtime = new ElapsedTime();  //run time
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime jamTimer = new ElapsedTime();
    private final ElapsedTime debounceTimer = new ElapsedTime();

    //============= INTAKE MOTOR CONSTANTS ===================
    public static final double INTAKE_TICKS_PER_REV = 145.1;   // example GoBilda 1150 motor
    public static final double INTAKE_RPM_CONVERSION = 60.0 / INTAKE_TICKS_PER_REV;

    //============= INTAKE VARIABLES ==========================
    private double intake_rpm;
    private INTAKEBALLSTATE state = INTAKEBALLSTATE.INTAKE_READY;

    //============= SPINDEXER & BALLS =========================
    private final double[] slotAngles;
    private List<Ball> balls = new ArrayList<>();
    private int currentSlot = 0;
    private int nextSlot;
    private int previousSlot = 0;

    //============= COLOR DETECTION ===========================
    private final ColorDetection colorDetection;
    private BallColor detectedColor = BallColor.UNKNOWN;
    private boolean colorDetected = false;

    // --- Constructor ---
    public IntakeBall(RobotHardware robot, GamepadEx gamepad, List<Ball> balls, double[] slotAngles) {
        this.robot = robot;
        this.gamepad1 = gamepad;
        this.balls = balls;
        this.slotAngles = slotAngles;

        this.colorDetection = new ColorDetection(robot);
        this.detectedColor = BallColor.UNKNOWN;
        this.robot.spindexerServo.setPosition(slotAngles[0]);

        timer.reset();
    }

    // --- FSM Update Loop ---
    public void IntakeBallUpdate() {
        /// === Read velocity in ticks/sec and convert to RPM ===
        double intake_ticksPerSec = robot.intakeMotor.getVelocity();
        intake_rpm = intake_ticksPerSec * INTAKE_RPM_CONVERSION;

        /// check for jam
        boolean jammed = isJammed();

        /// update color detection once per loop
        colorDetection.updateDetection();

        // Reverse slot position to previous one
        if (gamepad1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            state = INTAKEBALLSTATE.INTAKE_UNJAMMING;
            timer.reset();
        }

        // FSM STATES
        switch (state) {
            case INTAKE_READY:
                handleReadyState();
                break;

            case INTAKE_SWEEPING:
                handleSweepingState(jammed);
                break;

            case INTAKE_DETECTED:
                handleDetectedState();
                break;

            case INTAKE_INDEXING:
                handleIndexingState();
                break;

            case INTAKE_FULL:
                stopIntake();
                break;

            case INTAKE_UNJAMMING:
                handleUnjammingState();
                break;
        }
    }

    //======================== FSM HANDLERS ========================

    private void handleReadyState() {
        currentSlot = findEmptySlot();
        if (currentSlot == -1) currentSlot = 0;

        robot.spindexerServo.setPosition(slotAngles[currentSlot]);
        robot.leftGateServo.setPosition(RobotActionConfig.gateUp);
        robot.rightGateServo.setPosition(RobotActionConfig.gateUp);

        if (gamepad1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            robot.intakeMotor.setPower(0.6);
            state = INTAKEBALLSTATE.INTAKE_SWEEPING;
            timer.reset();
        }
    }

    private void handleSweepingState(boolean jammed) {
        if (jammed) {
            robot.intakeMotor.setPower(-0.2);
            try {
                sleep(150);  // shorter to avoid slowdowns
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            robot.intakeMotor.setPower(0.0);
        }

        if (colorDetection.isBallPresent()) {
            robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
            robot.rightGateServo.setPosition(RobotActionConfig.gateDown);
            timer.reset();
            state = INTAKEBALLSTATE.INTAKE_DETECTED;
        }
    }

    private void handleDetectedState() {
        if (colorDetection.isColorStable()) {
            colorDetected = true;
            // Convert string result → enum
            detectedColor = BallColor.fromString(colorDetection.getStableColor());

            Ball b = balls.get(currentSlot);
            b.setHasBall(true);
            b.setBallColor(detectedColor);  // ultra-fast enum update

            stopIntake();
            nextSlot = findEmptySlot();

            if (!areAllSlotsFull()) {
                previousSlot = currentSlot;
                currentSlot = nextSlot;
                timer.reset();
                state = INTAKEBALLSTATE.INTAKE_INDEXING;
            } else {
                state = INTAKEBALLSTATE.INTAKE_FULL;
            }
        }
    }

    private void handleIndexingState() {
        double t = timer.seconds();

        if (t > 0.25) {
            robot.spindexerServo.setPosition(slotAngles[nextSlot]);
            colorDetected = false;
            robot.intakeMotor.setPower(0.6);
        }

        if (t > 0.5) {
            robot.leftGateServo.setPosition(RobotActionConfig.gateUp);
            robot.rightGateServo.setPosition(RobotActionConfig.gateUp);
            timer.reset();
            state = INTAKEBALLSTATE.INTAKE_SWEEPING;
        }
    }

    private void handleUnjammingState() {
        robot.spindexerServo.setPosition(slotAngles[previousSlot]);

        if (timer.seconds() > 0.5) {
            Ball b = balls.get(previousSlot);
            b.setHasBall(false);
            b.setBallColor(BallColor.UNKNOWN);
            currentSlot = previousSlot;
            timer.reset();
            state = INTAKEBALLSTATE.INTAKE_SWEEPING;
        }
    }

    //======================== HELPERS ========================

    public void stopIntake() {
        robot.intakeMotor.setPower(0.0);
    }

    private boolean isJammed() {
        if (robot.intakeMotor.getPower() > 0.2 && intake_rpm < RobotActionConfig.intakeRPM_THRESHOLD) {
            if (jamTimer.seconds() > 0.3) return true;
        } else {
            jamTimer.reset();
        }
        return false;
    }

    public boolean areAllSlotsFull() {
        return getNumberOfBalls() == balls.size();
    }

    public int findEmptySlot() {
        for (Ball b : balls) {
            if (!b.hasBall()) return b.getSlotPosition();
        }
        return -1;
    }

    public int getNumberOfBalls() {
        int count = 0;
        for (Ball b : balls) if (b.hasBall()) count++;
        return count;
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    //======================== GETTERS / SETTERS ========================

    public INTAKEBALLSTATE getState() { return state; }
    public boolean isReady() { return state == INTAKEBALLSTATE.INTAKE_READY; }
    public boolean isFull() { return state == INTAKEBALLSTATE.INTAKE_FULL; }
    public int getCurrentSlot() { return currentSlot; }
    public BallColor getDetectedColor() { return detectedColor; }
    public boolean isColorDetected() { return colorDetected; }
    public List<Ball> getBalls() { return balls; }

    public void setState(INTAKEBALLSTATE state) { this.state = state; }

    public void resetSpindexerSlot() {
        currentSlot = 0;
        robot.spindexerServo.setPosition(slotAngles[0]);
        for (Ball b : balls) b.reset();
    }
}
