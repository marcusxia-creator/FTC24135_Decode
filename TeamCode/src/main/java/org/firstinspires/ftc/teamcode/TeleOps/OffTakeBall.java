package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.stream.Collectors;

public class OffTakeBall {

    public enum OFFTAKEBALLSTATE {
        OFFTAKE_READY,
        OFFTake_Prep,
        OFFTAKE_SHOOTING,
        OFFTAKE_EJECTING,
        OFFTAKE_DONE
    }
    public enum SHOOTERSTATE{
        ON,
        OFF
    }

    private OFFTAKEBALLSTATE state = OFFTAKEBALLSTATE.OFFTAKE_READY;
    private SHOOTERSTATE shooterState = SHOOTERSTATE.OFF;

    private final ElapsedTime timer = new ElapsedTime();

    private final RobotHardware robot;
    private final GamepadEx gamepad2;

    //------ Shared ball system ------
    private final SlotList ballSlots;
    private List<SlotList.BallSlot> ballsWithBallSlot;
    private int targetBallSlot;

    // --- Color sequence logic (now enum-based) ---
    private BallColor[] targetSequence = {
            BallColor.UNKNOWN,  // adjust depending on your defined colors
            BallColor.UNKNOWN,
            BallColor.UNKNOWN
    };
    private boolean useColorSequence = true;
    private int currentColorTargetIndex = 0;    // for color based sequence indexing
    private int currentCounterIndex = 0;   // for non color based sequence indexing
    private int cycle_no = 0;      // counter for non color based sequence times
    
    // --- Pre-computed firing order (slot indices based on color sequence) ---
    private int[] firingColorOrder;  // stores slot indices in shooting order

    /// --- Shooter power table ---
    LUTPowerCalculator shooterDiscreteZonePowerTable;
    private double calculatedShootPower = 0;
    private double currentDistanceToGoal;

    // --- Constructor ---
    public OffTakeBall(RobotHardware robot, GamepadEx gamepad2, SlotList ballSlots, LUTPowerCalculator shooterTunablePowerTable) {
        this.robot = robot;
        this.gamepad2 = gamepad2;
        this.ballSlots = ballSlots;
        this.shooterDiscreteZonePowerTable = shooterTunablePowerTable;
    }

    // --- Main update loop ---
    public void update() {
        /// Method 1 get calculated shoot power from look up table.
        /// TODO  need to add the pid power calcualtion
        calculatedShootPower = shooterDiscreteZonePowerTable.getPower();
        setShooterPower(calculatedShootPower);

        switch (state) {
            case OFFTAKE_READY:
                handleOfftakeReady();
                break;

            case OFFTake_Prep:
                if (ballSlots.getBallCount()>0) {
                    targetBallSlot = ballSlots.findAnySlotWithBall();
                    robot.spindexerServo.setPosition(ballSlots.getSlot(targetBallSlot).getSlotAngle());
                    state = OFFTAKEBALLSTATE.OFFTAKE_SHOOTING;
                    timer.reset();
                }
                else {
                    state = OFFTAKEBALLSTATE.OFFTAKE_DONE;
                }
                break;

            case OFFTAKE_SHOOTING:
                handleShootingState();
                break;

            case OFFTAKE_EJECTING:
                handleEjectingState();
                break;

            case OFFTAKE_DONE:
                resetCycle();
                break;
        }
    }

    // ===============================================================
    // === FSM Helper methods ===
    // ===============================================================

    /** Handles spin-up, fire, and transition to ejecting. */
    private void handleOfftakeReady() {

        // start the shooter power
        setShooterState(SHOOTERSTATE.ON);

        // Prepare ball list
        ballsWithBallSlot = ballSlots.getSlotsReadOnly().stream()
                .filter(SlotList.BallSlot::hasBall)
                .collect(Collectors.toList());

        cycle_no = ballsWithBallSlot.size();
        currentColorTargetIndex = 0;
        currentCounterIndex = 0;
        timer.reset();

        // Determine mode for this cycle
        useColorSequence = !isUnknownSequence(targetSequence);

        // Pre-compute firing order based on target sequence
        if (useColorSequence) {
            firingColorOrder = computeColorFiringOrder();
        }

        state = OFFTAKEBALLSTATE.OFFTake_Prep;
    }
    /// handle shooting state
    private void handleShootingState() {
        double startTime = timer.seconds();;
        if (timer.seconds() > startTime+0.5) {
            robot.spindexerServo.setPosition(0);
            state = OFFTAKEBALLSTATE.OFFTAKE_EJECTING;
            timer.reset();
        }
    }
    /** Handles ejection timing and advancing index. */
    private void handleEjectingState() {
        resetCycle();
        /**
        ejectCurrentBall(ballSlots.getSlot(targetBallSlot));
         */
        if (timer.seconds() > EJECT_TIME) {
            if (useColorSequence) {
                currentColorTargetIndex++;
                if (currentColorTargetIndex < targetSequence.length) {
                    state = OFFTAKEBALLSTATE.OFFTake_Prep;
                } else {
                    state = OFFTAKEBALLSTATE.OFFTAKE_DONE;
                }
            } else {
                currentCounterIndex++;
                if (currentCounterIndex < cycle_no) {
                    state = OFFTAKEBALLSTATE.OFFTake_Prep;
                } else {
                    state = OFFTAKEBALLSTATE.OFFTAKE_DONE;
                }
            }
        }
    }

    // =============================================================
    // NEW: Add this public method to receive the distance from your TeleOp
    public void setDistanceToGoal(double distance) {
        this.currentDistanceToGoal = distance;
    }
    /**
     * Pre-compute which slot index to fire for each position in sequence
     * Returns array of slot indices matching the target color sequence
     */
    private int[] computeColorFiringOrder() {
        int[] order = new int[targetSequence.length];
        //int[] order = new int[cycle_no];

        // 1. Declare and initialize the boolean array here.
        // It will have the same size as your number of slots (e.g., 3) and be initialized to {false, false, false}.
        boolean[] slotHasBeenClaimed = new boolean[ballSlots.size()];
        //boolean [] = new boolean[cycle_no];

        for (int i = 0; i < targetSequence.length; i++) {
            /** Alternative way to get targetsequence in a %3 remaining loop for the
             * n ++
             * sequence_index = n%%3
             * BallColor targetColor = targetSequence[sequence_index];
            */
            BallColor targetColor = targetSequence[i];
            // 2. Declare a variable to hold the result of the search.
            int foundSlotIndex = findAvailableSlotByColor(targetColor, slotHasBeenClaimed);

            order[i] = foundSlotIndex;  // -1 if not found

            if (foundSlotIndex != -1){
                slotHasBeenClaimed[foundSlotIndex] = true;
            }
        }
        return order;
    }

    /**
     * Find the index of a slot containing the specified color ball
     * Returns -1 if not found
     */
    private int findAvailableSlotByColor(BallColor targetColor,boolean [] claimedSlots) {

        for (int i = 0; i < ballSlots.size(); i++) {
            // Check three conditions:
            // 1. Has this slot already been claimed? (if !false, then proceed)
            if (!claimedSlots[i]) {
                // This slot has not been claimed yet, so we can check it.
                SlotList.BallSlot slot = ballSlots.getSlot(i);
                // 2. Does this slot physically contain a ball?
                // 3. Does the ball in this slot match our target color?
                if (slot.hasBall() && slot.getColor() == targetColor) {
                    // It has a ball and it's the right color. This is our match.
                    return i;
                }
            }
            // If claimedSlots[i] is true, the outer 'if' fails and this slot is correctly skipped.
        }
        return -1;  // not found
    }

    // Note: findNextTargetBall() method removed - now using pre-computed firingOrder

    /** --- Eject current ball (mark slot empty) ---*/
    private void ejectCurrentBall(SlotList.BallSlot b) {
        if (b != null) {
            b.setHasBall(false);
            b.setBallColor(BallColor.UNKNOWN);
        }
    }

    // --- Reset shooter cycle manually ---
    private void resetCycle() {
        currentColorTargetIndex = 0;
        currentCounterIndex = 0;
        cycle_no = 0;
        useColorSequence = false;
        state = OFFTAKEBALLSTATE.OFFTAKE_READY;
        setShooterState(SHOOTERSTATE.OFF);
    }
    // --- Set Shooter Power ---
    public void setShooterPower(double power) {
        if (shooterState == SHOOTERSTATE.ON) {
            robot.topShooterMotor.setPower(power);
            robot.bottomShooterMotor.setPower(power);
        } else {
            robot.topShooterMotor.setPower(0.0);
            robot.bottomShooterMotor.setPower(0.0);
        }
    }
    // --- Set Shooter State ---
    public void setShooterState(SHOOTERSTATE newState) {
        this.shooterState = newState;
    }

    // --- Public API ---
    public boolean isSortingComplete() {
        return state == OFFTAKEBALLSTATE.OFFTAKE_DONE;
    }

    public OFFTAKEBALLSTATE getState() {
        return state;
    }

    public void setSequence(BallColor[] sequence) {
        this.targetSequence = sequence;
        currentColorTargetIndex = 0;
    }

    public void setState(OFFTAKEBALLSTATE newState) {
        this.state = newState;
    }

    public boolean isUnknownSequence(BallColor[] sequence) {
        if (sequence == null || sequence.length == 0) return true;

        for (BallColor color : sequence) {
            if (color != BallColor.UNKNOWN) {
                return false; // found a known color
            }
        }
        return true; // all are UNKNOWN
    }
    public boolean isColorSequence() {
        return useColorSequence;
    }

    public double getShooterPower(){return calculatedShootPower;}
}


