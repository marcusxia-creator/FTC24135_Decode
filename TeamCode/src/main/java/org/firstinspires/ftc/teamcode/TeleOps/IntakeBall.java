package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class IntakeBall {

    public enum INTAKEBALLSTATE {
        INTAKE_READY,
        INTAKE_SWEEPING,
        INTAKE_DETECTED,
        INTAKE_INDEXING,
        INTAKE_END,
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
    private double intake_RPM;
    private INTAKEBALLSTATE state = INTAKEBALLSTATE.INTAKE_READY;

    private final ElapsedTime sweepTimer = new ElapsedTime();
    private boolean reversing = false;

    //============= SPINDEXER & BALLS =========================
    private final double[] slotAngles;
    private List<BallSlot> ballSlots = new ArrayList<>();
    private final SlotList slotList;
    private int currentSlot = 0;
    private int nextSlot;
    private int previousSlot = 0;

    //============= COLOR DETECTION ===========================
    private final ColorDetection colorDetection;
    private BallColor detectedColor = BallColor.UNKNOWN;
    private boolean colorDetected = false;

    // --- Constructor ---
    public IntakeBall(RobotHardware robot, GamepadEx gamepad, SlotList slotList, double[] slotAngles) {
        this.robot = robot;
        this.gamepad1 = gamepad;
        this.slotList = slotList;
        this.slotAngles = slotAngles;

        this.colorDetection = new ColorDetection(robot);
        this.detectedColor = BallColor.UNKNOWN;
        this.robot.spindexerServo.setPosition(slotAngles[0]);

        timer.reset();
    }

    // --- FSM Update Loop ---
    public void update() {
        /// === Read velocity in ticks/sec and convert to RPM ===
        double intake_ticksPerSec = robot.intakeMotor.getVelocity();
        intake_RPM = intake_ticksPerSec * INTAKE_RPM_CONVERSION;

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

            case INTAKE_END:
                stopIntake();
                break;

            case INTAKE_UNJAMMING:
                handleUnjammingState();
                break;
        }
    }

    //======================== FSM HANDLERS ========================

    private void handleReadyState() {
        currentSlot = slotList.findNextEmptySlot();
        if (currentSlot == -1) currentSlot = 0;
        double slotAngle = slotList.getSlot(currentSlot).getSlotAngle();
        robot.spindexerServo.setPosition(slotAngle);
        if (timer.seconds() > 0.25) {
            //robot.intakeMotor.setPower(INTAKE_POWER);
            state = INTAKEBALLSTATE.INTAKE_SWEEPING;
            timer.reset();
        }
    }

    private void handleSweepingState(boolean jammed) {
        if (jammed && !reversing) {
            // start short reverse pulse
            reversing = true;
            sweepTimer.reset();
            robot.intakeMotor.setPower(INTAKE_REVERSE_POWER);
        }

        if (reversing) {
            // let it reverse for 0.3 s, then stop
            if (sweepTimer.seconds() > 0.3) {
                robot.intakeMotor.setPower(0.0);
                reversing = false;
            }
        }
        if (!jammed && !reversing) {
            // normal sweeping forward when not jammed
            robot.intakeMotor.setPower(INTAKE_POWER);
        }

        if(colorDetection.isBallPresent()){
            //robot.leftGateServo.setPosition(GATEDOWN);
            //robot.rightGateServo.setPosition(GATEDOWN);
            state = INTAKEBALLSTATE.INTAKE_DETECTED;
        }
    }

    private void handleDetectedState() {
        if (colorDetection.isColorStable()) {
            colorDetected = true;
            // Convert string result → enum
            detectedColor = colorDetection.getStableColor();

            slotList.setSlotColor(currentSlot,detectedColor);
            slotList.setSlotHasBall(currentSlot,colorDetected);

            stopIntake();
            nextSlot = slotList.findNextEmptySlot();

            if (!slotList.isFull()) {
                previousSlot = currentSlot;
                currentSlot = nextSlot;
                timer.reset();
                state = INTAKEBALLSTATE.INTAKE_INDEXING;
            } else {
                state = INTAKEBALLSTATE.INTAKE_END;
            }
        }
    }

    private void handleIndexingState() {
        double t = timer.seconds();

        if (t > 0.25) {
            robot.spindexerServo.setPosition(slotList.getSlot(currentSlot).getSlotAngle());
            colorDetected = false;
            robot.intakeMotor.setPower(INTAKE_POWER);

            timer.reset();
            state = INTAKEBALLSTATE.INTAKE_SWEEPING;
        }
    }

    private void handleUnjammingState() {
        double t = timer.seconds();
        // --- 1. Reverse intake roller early to clear jam ---
        if (t < 0.1) {
            // short reverse pulse
            robot.intakeMotor.setPower(-0.7);
        }
        else if (t < 0.5) {
            // pause motor briefly to let ball roll back
            robot.intakeMotor.setPower(0.0);
        }

        // --- 2. Return spindexer to previous slot to re-align ---
        if (t >= 0.5 && t < 0.75) {
            robot.spindexerServo.setPosition(slotList.getSlot(previousSlot).getSlotAngle());
        }

        // --- 3. After 1.0 s, mark the slot as cleared ---
        if (t >= 0.75 ) {
            slotList.setSlotHasBall(previousSlot,false);
            slotList.setSlotColor(previousSlot,BallColor.UNKNOWN);
            currentSlot = previousSlot;
            robot.intakeMotor.setPower(INTAKE_POWER);
            timer.reset();
            state = INTAKEBALLSTATE.INTAKE_SWEEPING;
        }
    }
    //======================== HELPERS ========================

    public void stopIntake() {
        robot.intakeMotor.setPower(0.0);
    }

    private boolean isJammed() {
        if (robot.intakeMotor.getPower() > 0.2 && intake_RPM < intakeRPM_THRESHOLD) {
            if (jamTimer.seconds() > 0.3) return true;
        } else {
            jamTimer.reset();
        }
        return false;
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    //======================== GETTERS / SETTERS ========================

    public INTAKEBALLSTATE getState() { return state; }
    public boolean isReady() { return state == INTAKEBALLSTATE.INTAKE_READY; }
    public boolean isFull() { return state == INTAKEBALLSTATE.INTAKE_END; }
    public BallColor getDetectedColor() { return detectedColor; }
    public boolean isColorDetected() { return colorDetected; }

    public void setState(INTAKEBALLSTATE state) { this.state = state; }

}
