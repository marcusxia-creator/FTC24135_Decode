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
    private GamepadEx gamepad1;
    private List<Ball> balls;
    private double[] slotAngles;
    private int currentTargetIndex = 0;
    private List<String> requiredSequence = new ArrayList<>();
    private boolean sortingComplete = false;
    private ElapsedTime timer = new ElapsedTime();

    public void setState(OFFTAKEBALLSTATE offtakeballstate) {offTakeBallState = offtakeballstate;}
    public OFFTAKEBALLSTATE state() {return offTakeBallState;}

    public void stopShootBall() { robot.shooterMotor.setPower(0);}

    public enum OFFTAKEBALLSTATE {
        READY, SORT, SHOOT, EMPTY
    }

    private OFFTAKEBALLSTATE offTakeBallState = OFFTAKEBALLSTATE.READY;

    public OffTakeBall(RobotHardware robot, GamepadEx gamepad1, List<Ball> balls, double[] slotAngles) {
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.balls = balls != null ? balls : new ArrayList<>();
        this.slotAngles = slotAngles;
        this.requiredSequence = new ArrayList<>();  // ✅ FIX ADDED HERE
    }

    public void setRequiredSequence(List<String> sequence) {
        requiredSequence.clear();
        if (sequence != null) requiredSequence.addAll(sequence);
        currentTargetIndex = 0;
        sortingComplete = false;
        offTakeBallState = OFFTAKEBALLSTATE.READY;
    }

    public boolean isSortingComplete() { return sortingComplete; }

    public void update() {
        switch (offTakeBallState) {

            case READY:
                if (gamepad1.wasJustPressed(GamepadKeys.Button.Y)) {
                    timer.reset();
                    offTakeBallState = OFFTAKEBALLSTATE.SORT;
                }
                break;

            case SORT:
                if (currentTargetIndex >= requiredSequence.size()) {
                    sortingComplete = true;
                    offTakeBallState = OFFTAKEBALLSTATE.EMPTY;
                    break;
                }

                String targetColor = requiredSequence.isEmpty() ? null : requiredSequence.get(currentTargetIndex);
                Ball targetBall = (targetColor != null) ? findBallByColor(targetColor) : findBall();

                if (targetBall != null && targetBall.hasBall) {
                    rotateSpindexer(targetBall);
                    offTakeBallState = OFFTAKEBALLSTATE.SHOOT;
                    timer.reset();
                } else {
                    sortingComplete = true;
                    offTakeBallState = OFFTAKEBALLSTATE.EMPTY;
                }
                break;

            case SHOOT:
                if (timer.seconds() > 0.5) robot.shooterMotor.setPower(0.75);
                if (timer.seconds() > 1.5) robot.pushRampServo.setPosition(RobotActionConfig.rampUpPos);
                if (timer.seconds() > 2.5) {
                    robot.pushRampServo.setPosition(RobotActionConfig.rampResetPos);
                    robot.shooterMotor.setPower(0);
                    currentTargetIndex++;
                    offTakeBallState = OFFTAKEBALLSTATE.SORT;
                }
                break;

            case EMPTY:
                robot.shooterMotor.setPower(0);
                sortingComplete = true;
                break;
        }
    }

    private void rotateSpindexer(Ball targetBall) {
        if (targetBall != null) {
            robot.spindexerServo.setPosition(targetBall.slotAngle);
            targetBall.hasBall = false;
        }
    }

    private Ball findBallByColor(String color) {
        for (Ball b : balls)
            if (b.hasBall && b.ballColor.equalsIgnoreCase(color))
                return b;
        return null;
    }

    private Ball findBall() {
        for (Ball b : balls)
            if (b.hasBall) return b;
        return null;
    }


}
