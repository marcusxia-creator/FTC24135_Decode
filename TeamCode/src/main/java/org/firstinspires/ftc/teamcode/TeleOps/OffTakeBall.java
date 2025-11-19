package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.stream.Collectors;

public class OffTakeBall {

    public enum OFFTAKEBALLSTATE {
        OFFTAKE_IDLE,
        OFFTAKE_AIMING,
        OFFTAKE_SHOOTING,
        OFFTAKE_EJECTING,
        OFFTAKE_DONE
    }

    private OFFTAKEBALLSTATE state = OFFTAKEBALLSTATE.OFFTAKE_IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    private final RobotHardware robot;
    private final GamepadEx gamepad2;

    //------ Shared ball system ------
    private final List<BallSlot> ballSlots;
    private List<BallSlot> ballsWithBallSlot;
    private BallSlot targetBallSlot;

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
    ShooterPowerTable shooterPowerTable;
    private double calculatedShootPower = 0;
    private double currentDistanceToGoal;

    // --- Constructor ---
    public OffTakeBall(RobotHardware robot, GamepadEx gamepad2, List<BallSlot> ballSlots, ShooterPowerTable shooterPowerTable) {
        this.robot = robot;
        this.gamepad2 = gamepad2;
        this.ballSlots = ballSlots;
        this.shooterPowerTable = shooterPowerTable;
    }

    // --- Main update loop ---
    public void update() {

        switch (state) {

            case OFFTAKE_IDLE:
                robot.shooterMotor.setPower(0.0);
                handleOfftakeIdleState();
                break;

            case OFFTAKE_AIMING:
                targetBallSlot = getNextTargetBall();
                if (targetBallSlot != null) {
                    robot.spindexerServo.setPosition(targetBallSlot.getSlotAngle());
                    state = OFFTAKEBALLSTATE.OFFTAKE_SHOOTING;
                    timer.reset();
                } else {
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
                if (gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                    resetCycle();
                }
                break;
        }
    }

    // ===============================================================
    // === FSM Helper methods ===
    // ===============================================================
    private void handleOfftakeIdleState() {
        if (gamepad2.getButton(GamepadKeys.Button.Y)) {
            //Close the gate
            robot.leftGateServo.setPosition(GATEDOWN);
            robot.rightGateServo.setPosition(GATEDOWN);
            // Prepare ball list
            ballsWithBallSlot = ballSlots.stream()
                    .filter(BallSlot::hasBall)
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
            
            state = OFFTAKEBALLSTATE.OFFTAKE_AIMING;
        }
    }

    /** Handles spin-up, fire, and transition to ejecting. */
    // NEW: Add this public method to receive the distance from your TeleOp
    public void setDistanceToGoal(double distance) {
        this.currentDistanceToGoal = distance;
    }
    private void handleShootingState() {
        double startTime;
        /// Method 1 get calculated shoot power from look up table.
        calculatedShootPower = shooterPowerTable.getPower(currentDistanceToGoal);

        /// Method 2 get calculated shoot power based on power curve fitting formula.
        //calculatedShootPower = -14.88+0.7736667*distance-0.01416667*Math.pow(distance,2)+0.000113333*Math.pow(distance,3)-3.33333e-7*Math.pow(distance,4);

        if (currentCounterIndex == 0){
            startTime = RAMP_UP_TIME_1st;
            calculatedShootPower = Range.clip(calculatedShootPower+0.1, 0.5, 1.0);
        }
        else{
            startTime = RAMP_UP_TIME;
        }
        robot.shooterMotor.setPower(calculatedShootPower);
        if (timer.seconds() > startTime-0.2) {
            robot.leftGateServo.setPosition(GATEUP);
            robot.rightGateServo.setPosition(GATEUP);
        }

        if (timer.seconds() > startTime) {
            robot.pushRampServo.setPosition(RAMP_UP);
        }

        if (timer.seconds() > startTime+0.5) {
            robot.shooterMotor.setPower(0.0);
            robot.pushRampServo.setPosition(RAMP_RESET_POSITION);
            robot.leftGateServo.setPosition(GATEDOWN);
            robot.rightGateServo.setPosition(GATEDOWN);
            state = OFFTAKEBALLSTATE.OFFTAKE_EJECTING;
            timer.reset();
        }
    }
    /** Handles ejection timing and advancing index. */
    private void handleEjectingState() {
        ejectCurrentBall(targetBallSlot);
        if (timer.seconds() > EJECT_TIME) {
            if (useColorSequence) {
                currentColorTargetIndex++;
                if (currentColorTargetIndex < targetSequence.length) {
                    state = OFFTAKEBALLSTATE.OFFTAKE_AIMING;
                } else {
                    state = OFFTAKEBALLSTATE.OFFTAKE_DONE;
                }
            } else {
                currentCounterIndex++;
                if (currentCounterIndex < cycle_no) {
                    state = OFFTAKEBALLSTATE.OFFTAKE_AIMING;
                } else {
                    state = OFFTAKEBALLSTATE.OFFTAKE_DONE;
                }
            }
        }
    }

    // =============================================================
    /**
     * Pre-compute which slot index to fire for each position in sequence
     * Returns array of slot indices matching the target color sequence
     */
    private int[] computeColorFiringOrder() {
        int[] order = new int[targetSequence.length];

        // 1. Declare and initialize the boolean array here.
        // It will have the same size as your number of slots (e.g., 3) and be initialized to {false, false, false}.
        boolean[] slotHasBeenClaimed = new boolean[ballSlots.size()];
        
        for (int i = 0; i < targetSequence.length; i++) {
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
                BallSlot slot = ballSlots.get(i);
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
    
    // =============================================================
    /** Selects the next target ball based on current mode. */
    private BallSlot getNextTargetBall() {
        if (useColorSequence) {
            // Tag-guided mode - use pre-computed firing order
            if (currentColorTargetIndex < firingColorOrder.length) {
                int slotIndex = firingColorOrder[currentColorTargetIndex];
                if (slotIndex >= 0) {
                    return ballSlots.get(slotIndex);
                }
                return null;  // color not found
            }
        } else {
            // Sequential (free shooting) mode
            if (currentCounterIndex < cycle_no) {
                return ballsWithBallSlot.get(cycle_no - currentCounterIndex - 1); //reverse order for shooting
            }
        }
        return null;
    }
    
    // Note: findNextTargetBall() method removed - now using pre-computed firingOrder
    
    /** --- Eject current ball (mark slot empty) ---*/
    private void ejectCurrentBall(BallSlot b) {
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
        state = OFFTAKEBALLSTATE.OFFTAKE_IDLE;
        robot.shooterMotor.setPower(0.0);
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


