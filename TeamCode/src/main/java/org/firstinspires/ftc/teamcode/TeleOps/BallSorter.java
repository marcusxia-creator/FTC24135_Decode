package org.firstinspires.ftc.teamcode.TeleOps;

import java.util.ArrayList;
import java.util.List;

public class BallSorter {
    private RobotHardware robot;
    private IntakeBall intakeBall;
    private List<Ball> balls;   // reference from IntakeBall
    private double[] slotAngles;
    private int currentTargetIndex = 0;
    private List<String> requiredSequence;
    private boolean sortingComplete = false;

    public BallSorter(RobotHardware robot, List<Ball> balls, double[] slotAngles) {
        this.robot = robot;
        this.balls = balls;
        this.slotAngles = slotAngles;
        this.requiredSequence = new ArrayList<>();
    }

    /** Define the required off-take sequence such as ["Purple", "Green", "Purple"] */
    public void setRequiredSequence(List<String> sequence) {
        this.requiredSequence.clear();
        this.requiredSequence.addAll(sequence);
        currentTargetIndex = 0;
        sortingComplete = false;
    }

    /** Check whether all required colors have been off-taken */
    public boolean isSortingComplete() {
        return sortingComplete;
    }

    /** Main update loop – call periodically (for example from TeleOp loop) */
    public void update() {
        if (sortingComplete || requiredSequence.isEmpty()) return;

        if (currentTargetIndex >= requiredSequence.size()) {
            sortingComplete = true;
            intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_START);
            return;
        }

        String targetColor = requiredSequence.get(currentTargetIndex);
        Ball targetBall = findBallByColor(targetColor);

        if (targetBall != null) {
            // Rotate spindexer to ball slot angle
            robot.spindexerServo.setPosition(targetBall.slotAngle);

            // Mark ball as removed
            targetBall.hasBall = false;

            // Move to next color
            currentTargetIndex++;

            if (currentTargetIndex >= requiredSequence.size()) {
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

    /** Example off-take: reverse intake motor for short time */
    private void ejectBall(Ball ball) {
        // Rotate already done before calling
        robot.intakeMotor.setPower(-0.6);
        try {
            Thread.sleep(500);  // run motor backward 0.5s
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        robot.intakeMotor.setPower(0);
    }
}
