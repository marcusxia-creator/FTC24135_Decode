package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class OffTakeBall {
    private RobotHardware robot;
    private IntakeBall intakeBall;
    private ColorDetection colorDetection;
    private GamepadEx gamepad1;
    private AprilTagUpdate AprilTagUpdate;
    private List<Ball> balls;   // reference from IntakeBall
    private Ball targetBall;
    private double[] slotAngles;
    private int currentTargetIndex = 0;
    private List<String> requiredSequence;
    private boolean sortingComplete = false;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime();

    /** DEFINE OFFTAKEBALLSTATE*/
    public enum OFFTAKEBALLSTATE {
        READY,
        FLOW,
        SORT,
        SHOOT,
        EMPTY
    }
    /** SET INITIAL OFFTAKEBALLSTATE*/
    private OFFTAKEBALLSTATE offTakeBallState = OFFTAKEBALLSTATE.READY;

    // --- Constructor ---
    public OffTakeBall(RobotHardware robot, GamepadEx gamepad1,List<Ball> balls, double[] slotAngles) {
        this.robot = robot;
        this.balls = balls;
        this.slotAngles = slotAngles;
        this.gamepad1 = gamepad1;
        this.colorDetection = new ColorDetection(robot);
    }

    /**
     * Define the required off-take sequence such as ["Purple", "Green", "Purple"]
     */
    public void setRequiredSequence(List<String> sequence) {
        requiredSequence.clear();
        requiredSequence.addAll(sequence);
        currentTargetIndex = 0;
        sortingComplete = false;
    }
    /**
     * Check whether all required colors have been off-taken
     */
    public boolean isSortingComplete() {
        return sortingComplete;
    }

    /**
     * Executes the main logic for the off-take process.
     * This method should be called periodically, for example, within the main TeleOp loop.
     * <p>
     * It checks if the off-take sequence is complete or empty. If not, it identifies the
     * next required ball color from the sequence. If a ball of the target color is found,
     * it rotates the spindexer to that ball's position, marks the ball as removed, and
     * advances to the next target in the sequence.
     * <p>
     * The process is marked as complete if all balls in the sequence are processed or if
     * a required ball color cannot be found in the current inventory.
     */
    public void update() {

        switch (offTakeBallState) {
            case READY:
                if (gamepad1.getButton(GamepadKeys.Button.Y)) {
                    timer.reset();
                    offTakeBallState = OFFTAKEBALLSTATE.SORT;
                }
                break;

            case SORT:
                if (requiredSequence != null && !requiredSequence.isEmpty()) {
                    // safe to use, and has elements
                    String targetColor = requiredSequence.get(currentTargetIndex);
                    targetBall = findBallByColor(targetColor);
                    if (currentTargetIndex >= requiredSequence.size()) {
                        sortingComplete = true;
                        intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_READY);
                        offTakeBallState = OFFTAKEBALLSTATE.EMPTY;
                    }
                    if (targetBall.hasBall && targetBall != null) {
                        rotateSpindexer(targetBall);
                        offTakeBallState = OFFTAKEBALLSTATE.SHOOT;
                        timer.reset();
                    } else {
                        sortingComplete = true;
                        offTakeBallState = OFFTAKEBALLSTATE.READY;
                    }
                } else {
                    targetBall = findBall();
                    if (targetBall.hasBall && targetBall != null) {
                        rotateSpindexer(targetBall);
                        offTakeBallState = OFFTAKEBALLSTATE.SHOOT;
                        timer.reset();
                    }else{
                        sortingComplete = true;
                        offTakeBallState = OFFTAKEBALLSTATE.READY;
                    }
                }
                break;

            case SHOOT:
                if (targetBall != null) {
                    if (timer.seconds() > 0.5) {
                        shootBall(0.75);
                    }
                    if (timer.seconds() > 1.5) {
                        robot.pushRampServo.setPosition(RobotActionConfig.rampUpPos);
                    }
                    if (timer.seconds() > 2.5){
                        robot.pushRampServo.setPosition(RobotActionConfig.rampResetPos);
                    }
                } else {
                    sortingComplete = true;
                    offTakeBallState = OFFTAKEBALLSTATE.EMPTY;
                }
                break;
            case EMPTY:
                robot.shooterMotor.setPower(0);
                intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_READY);
                break;
            default:
                break;
        }
    }

    /** Helper to rotate the spindexer*/
   private void rotateSpindexer(Ball targetBall){
        if (targetBall != null) {
            // Rotate spindexer to ball slot angle
            robot.spindexerServo.setPosition(targetBall.slotAngle);

            // Mark ball as removed
            targetBall.hasBall = false;

            // Move to next color
            currentTargetIndex++;

            if (currentTargetIndex >= 3) {
                sortingComplete = true;
            }
        } else {
            // If color not found among current balls, stop or skip
            sortingComplete = true;
        }
   }

    /** Helper to find the first ball matching the target color that still exists */
    private Ball findBallByColor(String color) {
        for (Ball b : balls) {
            if (b.hasBall && b.ballColor.equalsIgnoreCase(color)) {
                return b;
            }
        }
        return null;
    }
   /** Helper to find the first ball hasBall is True*/
    private Ball findBall (){
         for ( Ball b :balls) {
            if (b.hasBall ){ return b;}
         }
    return null;
}
    /** Example off-take: reverse intake motor for short time */
    private void shootBall(double power) {
        // Rotate already done before calling
        robot.shooterMotor.setPower(power);
    }
    public void stopShootBall(){
        // Stop shooter
        robot.shooterMotor.setPower(0);
    }

    /** Helper ButtonDebounce */
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    // --- Getters ---
    public OFFTAKEBALLSTATE getOffTakeBallState() {
        return offTakeBallState;
    }
    public void setState(OFFTAKEBALLSTATE state) {offTakeBallState = state; }
    public boolean isReady() { return offTakeBallState == OFFTAKEBALLSTATE.READY; }
    public OFFTAKEBALLSTATE state() { return offTakeBallState;}
    public boolean isEmpty() { return offTakeBallState == OFFTAKEBALLSTATE.EMPTY; }
    public List<Ball> getBalls() { return balls; }

}
