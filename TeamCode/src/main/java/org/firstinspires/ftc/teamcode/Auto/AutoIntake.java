package org.firstinspires.ftc.teamcode.Auto;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOps.Ball;
import org.firstinspires.ftc.teamcode.TeleOps.BallColor;
import org.firstinspires.ftc.teamcode.TeleOps.ColorDetection;
import org.firstinspires.ftc.teamcode.TeleOps.IntakeBall;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import java.util.ArrayList;
import java.util.List;

public class AutoIntake {
    public enum INTAKEBALLSTATE {
        INTAKE_READY,
        INTAKE_SWEEPING,
        INTAKE_DETECTED,
        INTAKE_INDEXING,
        INTAKE_FULL,
        INTAKE_UNJAMMING
    }

    //============= SUBSYSTEMS ===============================
    private final RobotHardware robot;

    //============= TIMERS ===================================
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime jamTimer = new ElapsedTime();

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
    public AutoIntake(RobotHardware robot, List<Ball> balls, double[] slotAngles) {
        this.robot = robot;
        this.balls = balls;
        this.slotAngles = slotAngles;

        this.colorDetection = new ColorDetection(robot);
        this.detectedColor = BallColor.UNKNOWN;
        this.robot.spindexerServo.setPosition(slotAngles[0]);
        timer.reset();
    }

    //=============================================================
    // AUTO ENTRY POINTS
    //=============================================================

    /** Start the FSM for auto intake (instead of gamepad trigger). */
    public void startAutoIntake() {
        if (state == INTAKEBALLSTATE.INTAKE_READY || state == INTAKEBALLSTATE.INTAKE_FULL) {
            currentSlot = findEmptySlot();
            if (currentSlot == -1) currentSlot = 0;
            robot.spindexerServo.setPosition(slotAngles[currentSlot]);
            robot.leftGateServo.setPosition(GATEUP);
            robot.rightGateServo.setPosition(GATEUP);
            robot.intakeMotor.setPower(0.6);
            state = INTAKEBALLSTATE.INTAKE_SWEEPING;
            timer.reset();
        }
    }

    /** Stop intake gracefully. */
    public void stopIntake() {
        robot.intakeMotor.setPower(0.0);
    }

    //=============================================================
    // FSM MAIN LOOP (no gamepad needed)
    //=============================================================

    public void IntakeBallUpdate() {
        // Update sensor data
        colorDetection.updateDetection();
        double intake_ticksPerSec = robot.intakeMotor.getVelocity();
        intake_rpm = intake_ticksPerSec * (60.0 / 145.1);

        boolean jammed = isJammed();

        // FSM STATE MACHINE
        switch (state) {
            case INTAKE_READY:
                // nothing automatic here — must call startAutoIntake()
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

    //=============================================================
    // FSM HANDLERS (unchanged from your version)
    //=============================================================

    private void handleSweepingState(boolean jammed) {
        if (jammed) {
            robot.intakeMotor.setPower(-0.2);
            try {
                sleep(150);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            robot.intakeMotor.setPower(0.0);
        }

        if (colorDetection.isBallPresent()) {
            robot.leftGateServo.setPosition(GATEDOWN);
            robot.rightGateServo.setPosition(GATEDOWN);
            timer.reset();
            state = INTAKEBALLSTATE.INTAKE_DETECTED;
        }
    }

    private void handleDetectedState() {
        if (colorDetection.isColorStable()) {
            colorDetected = true;
            detectedColor = colorDetection.getStableColor();

            Ball b = balls.get(currentSlot);
            b.setHasBall(true);
            b.setBallColor(detectedColor);

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
            robot.leftGateServo.setPosition(GATEUP);
            robot.rightGateServo.setPosition(GATEUP);
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

    //=============================================================
    // HELPERS
    //=============================================================
    private boolean isJammed() {
        if (robot.intakeMotor.getPower() > 0.2 && intake_rpm < intakeRPM_THRESHOLD) {
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
        for (Ball b : balls) if (!b.hasBall()) return b.getSlotPosition();
        return -1;
    }

    public int getNumberOfBalls() {
        int count = 0;
        for (Ball b : balls) if (b.hasBall()) count++;
        return count;
    }

    //=============================================================
    // ROADRUNNER ACTION INTEGRATION
    //=============================================================

    /** RoadRunner Action wrapper for auto mode. */
    public Action collectBallsAction(int targetCount) {
        startAutoIntake();  // begin FSM

        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                IntakeBallUpdate();
                packet.put("FSM State", getState().toString());
                packet.put("Detected Color", getDetectedColor().toString());
                packet.put("Balls Collected", getNumberOfBalls());

                if (getNumberOfBalls() >= targetCount) {
                    stopIntake();
                    packet.put("Status", "Intake complete");
                    return false; // Action done
                }
                return true; // keep running
            }
        };
    }

    //======================== GETTERS / SETTERS ========================

    public AutoIntake.INTAKEBALLSTATE getState() { return state; }
    public int getCurrentSlot() { return currentSlot; }
    public BallColor getDetectedColor() { return detectedColor; }
    public boolean isColorDetected() { return colorDetected; }
    public List<Ball> getBalls() { return balls; }

    public void setState(AutoIntake.INTAKEBALLSTATE state) { this.state = state; }

    public void resetSpindexerSlot() {
        currentSlot = 0;
        robot.spindexerServo.setPosition(slotAngles[0]);
        for (Ball b : balls) b.reset();
    }
}

