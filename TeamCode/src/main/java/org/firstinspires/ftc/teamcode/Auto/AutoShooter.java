package org.firstinspires.ftc.teamcode.Auto;public class AutoShooter {



import androidx.annotation.NonNull;    public enum SHOOTERSTATE {

        SHOOTER_SPINUP,

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;        SHOOTER_SHOOTING,

import com.acmerobotics.roadrunner.Action;        SHOOTER_IDLE

import com.qualcomm.robotcore.util.ElapsedTime;    }



import org.firstinspires.ftc.teamcode.TeleOps.BallColor;    private SHOOTERSTATE state = SHOOTERSTATE.SHOOTER_IDLE;

import org.firstinspires.ftc.teamcode.TeleOps.BallSlot;    private ElapsedTime timer = new ElapsedTime();

import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;    private final RobotHardware robot;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;    private final SharedColorSequence sharedColorSequence;

    private double shooter_rpm;

import java.util.List;    // --- Constructor ---

    public AutoShooter(RobotHardware robot, SharedColorSequence sharedColorSequence) {

/**        this.robot = robot;

 * AutoShooter - Shoots balls in the order specified by an AprilTag color sequence.        this.sharedColorSequence = sharedColorSequence;

 * Integrates with RoadRunner 1.0 as an Action.    }       

 */

public class AutoShooter {    

    

    private enum ShooterState {}

        SPIN_UP,
        FIRE_SEQUENCE,
        FIRE_REMAINING,
        COMPLETE
    }
    
    private final RobotHardware robot;
    private final List<BallSlot> ballSlots;
    private final BallColor[] desiredSequence;
    private final ElapsedTime timer = new ElapsedTime();
    
    private ShooterState state = ShooterState.SPIN_UP;
    private int sequenceIndex = 0;
    private boolean[] slotFired;
    
    // Pre-sorted firing order: slot indices to fire in sequence
    private int[] firingOrder;
    
    // Timing parameters
    private static final double SPIN_UP_TIME = 0.5;      // Time to spin up shooter
    private static final double FIRE_DELAY = 0.35;       // Delay between each ball for spindexer rotation
    
    /**
     * Constructor
     * @param robot RobotHardware instance
     * @param ballSlots List of ball slots from spindexer
     * @param desiredSequence Color sequence from AprilTag (can be null/empty)
     */
    public AutoShooter(RobotHardware robot, List<BallSlot> ballSlots, BallColor[] desiredSequence) {
        this.robot = robot;
        this.ballSlots = ballSlots;
        this.desiredSequence = (desiredSequence != null) ? desiredSequence : new BallColor[0];
        this.slotFired = new boolean[ballSlots.size()];
        
        // Pre-compute firing order based on sequence
        this.firingOrder = computeFiringOrder();
    }
    
    /**
     * Pre-compute which slot index to fire for each position in sequence
     * Returns array of slot indices: [slot_for_ball_1, slot_for_ball_2, slot_for_ball_3]
     */
    private int[] computeFiringOrder() {
        int[] order = new int[3];
        
        for (int i = 0; i < 3; i++) {
            if (i < desiredSequence.length) {
                BallColor targetColor = desiredSequence[i];
                int slotIndex = findMatchingSlot(targetColor);
                order[i] = slotIndex; // -1 if not found
            } else {
                order[i] = -1; // No sequence specified
            }
        }
        
        return order;
    }
    
    /**
     * Create a RoadRunner Action for shooting
     */
    public Action shootAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return updateShooting(packet);
            }
        };
    }
    
    /**
     * Main FSM update - returns true while shooting, false when complete
     */
    private boolean updateShooting(TelemetryPacket packet) {
        double elapsed = timer.seconds();
        
        switch (state) {
            case SPIN_UP:
                if (elapsed == 0) {
                    // First call - start spin up shooter
                    robot.shooterMotor.setPower(RobotActionConfig.SHOOTER_POWER);
                    robot.leftGateServo.setPosition(RobotActionConfig.GATEDOWN);
                    robot.rightGateServo.setPosition(RobotActionConfig.GATEDOWN);
                    timer.reset();
                    packet.put("Shooter", "Spinning up...");
                }
                
                if (elapsed >= SPIN_UP_TIME) {
                    sequenceIndex = 0;
                    state = ShooterState.FIRE_SEQUENCE;
                    timer.reset();
                    packet.put("Shooter", "Ready to fire");
                }
                return true;
                
            case FIRE_SEQUENCE:
                // Check if we've fired all 3 balls
                if (sequenceIndex >= 3) {
                    state = ShooterState.COMPLETE;
                    timer.reset();
                    packet.put("Shooter", "All 3 balls fired");
                    return true;
                }
                
                // On first call for this ball, turn spindexer to position
                if (elapsed == 0) {
                    // Get pre-computed slot index for this sequence position
                    int slotIndex = firingOrder[sequenceIndex];
                    
                    if (slotIndex >= 0) {
                        // Turn spindexer to this slot
                        BallColor targetColor = desiredSequence[sequenceIndex];
                        robot.spindexerServo.setPosition(ballSlots.get(slotIndex).getSlotAngle());
                        packet.put("Turning to Ball " + (sequenceIndex + 1), targetColor.name() + " at slot " + slotIndex);
                    } else {
                        BallColor targetColor = desiredSequence[sequenceIndex];
                        packet.put("Skipped Ball " + (sequenceIndex + 1), targetColor.name() + " (not found)");
                        // Skip to next ball immediately
                        sequenceIndex++;
                        timer.reset();
                        return true;
                    }
                }
                
                // After delay, mark ball as fired and move to next
                if (elapsed >= FIRE_DELAY) {
                    int slotIndex = firingOrder[sequenceIndex];
                    
                    if (slotIndex >= 0) {
                        // Mark as fired (ball has been shot)
                        BallColor targetColor = desiredSequence[sequenceIndex];
                        slotFired[slotIndex] = true;
                        ballSlots.get(slotIndex).setHasBall(false);
                        ballSlots.get(slotIndex).setBallColor(BallColor.UNKNOWN);
                        packet.put("Fired Ball " + (sequenceIndex + 1), targetColor.name());
                    }
                    
                    sequenceIndex++;
                    timer.reset();
                }
                
                packet.put("Progress", sequenceIndex + "/3 balls");
                return true;
                
            case COMPLETE:
                if (elapsed >= 0.5) {
                    // Shutdown
                    robot.shooterMotor.setPower(0.0);
                    robot.leftGateServo.setPosition(RobotActionConfig.GATEDOWN);
                    robot.rightGateServo.setPosition(RobotActionConfig.GATEDOWN);
                    packet.put("Shooter", "Complete - motors stopped");
                    return false; // Action complete
                }
                packet.put("Shooter", "Shutting down...");
                return true;
                
            case FIRE_REMAINING:
                // Not used in simplified version
                return false;
        }
        
        return false;
    }
    
    /**
     * Find the first unfired slot matching the target color
     */
    private int findMatchingSlot(BallColor targetColor) {
        for (int i = 0; i < ballSlots.size(); i++) {
            BallSlot slot = ballSlots.get(i);
            if (!slotFired[i] && slot.hasBall() && slot.getColor() == targetColor) {
                return i;
            }
        }
        return -1;
    }
    
    /**
     * Fire a specific slot
     */
    private void fireSlot(int slotIndex, TelemetryPacket packet) {
        BallSlot slot = ballSlots.get(slotIndex);
        
        // Position spindexer to this slot
        robot.spindexerServo.setPosition(slot.getSlotAngle());
        
        // Mark as fired
        slotFired[slotIndex] = true;
        
        // Clear the slot
        slot.setHasBall(false);
        slot.setBallColor(BallColor.UNKNOWN);
        
        packet.put("Firing Slot", slotIndex);
        packet.put("Slot Angle", slot.getSlotAngle());
    }
    
    /**
     * Manual blocking shoot (simplified version)
     */
    public void shootBlocking() {
        // Spin up shooter
        robot.shooterMotor.setPower(RobotActionConfig.SHOOTER_POWER);
        robot.leftGateServo.setPosition(RobotActionConfig.GATEUP);
        robot.rightGateServo.setPosition(RobotActionConfig.GATEUP);
        sleep((long)(SPIN_UP_TIME * 1000));
        
        // Fire 3 balls according to pre-computed firing order
        for (int i = 0; i < 3; i++) {
            int slotIndex = firingOrder[i];
            
            if (slotIndex >= 0) {
                // Turn to slot and fire
                robot.spindexerServo.setPosition(ballSlots.get(slotIndex).getSlotAngle());
                slotFired[slotIndex] = true;
                ballSlots.get(slotIndex).setHasBall(false);
                ballSlots.get(slotIndex).setBallColor(BallColor.UNKNOWN);
            }
            
            // Delay between balls for spindexer to rotate
            sleep((long)(FIRE_DELAY * 1000));
        }
        
        // Shutdown
        robot.shooterMotor.setPower(0.0);
        robot.leftGateServo.setPosition(RobotActionConfig.GATEDOWN);
        robot.rightGateServo.setPosition(RobotActionConfig.GATEDOWN);
    }
    
    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
