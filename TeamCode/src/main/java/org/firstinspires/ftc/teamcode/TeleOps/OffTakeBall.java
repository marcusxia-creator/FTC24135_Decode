package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class OffTakeBall {

    public enum OFFTAKEBALLSTATE {
        OFFTAKE_IDLE,
        OFFTAKE_AIMING,
        OFFTAKE_SHOOTING,
        OFFTAKE_EJECTING,
        OFFTAKE_DONE
    }
    private OFFTAKEBALLSTATE state = OFFTAKEBALLSTATE.OFFTAKE_IDLE;
    private ElapsedTime timer = new ElapsedTime();

    private RobotHardware robot;
    private GamepadEx gamepad2;

    //------Ball System
    private List<Ball> balls; // shared balls list
    private Ball targetBall;

    // --- color sequence logic ---
    private String[] targetSequence = {"Purple", "Green","Purple"}; // example priority
    private int currentTargetIndex = 0;

    // --- constructor ---
    public OffTakeBall(RobotHardware robot, GamepadEx gamepad2, List<Ball> balls) {
        this.robot = robot;
        this.gamepad2 = gamepad2;
        this.balls = balls;
    }

    public void update() {
        switch (state) {
            case OFFTAKE_IDLE:
                robot.shooterMotor.setPower(0.0);
                if (gamepad2.getButton(GamepadKeys.Button.A)) {
                    // Start the shooting cycle
                    state = OFFTAKEBALLSTATE.OFFTAKE_AIMING;
                    timer.reset();
                }
                break;

            case OFFTAKE_AIMING:
                targetBall = findNextTargetBall();
                if (targetBall != null) {
                    // Move spindexer to that slot
                    robot.spindexerServo.setPosition(targetBall.getSlotAngle());
                    state = OFFTAKEBALLSTATE.OFFTAKE_SHOOTING;
                    timer.reset();
                } else {
                    // No matching color ball found
                    state = OFFTAKEBALLSTATE.OFFTAKE_DONE;
                }
                break;

            case OFFTAKE_SHOOTING:
                if (timer.seconds() > 0.1) {
                    // Example shooting logic (you can modify as needed)
                    robot.shooterMotor.setPower(0.75);
                    if (timer.seconds() > 0.5) {
                        robot.pushRampServo.setPosition(RobotActionConfig.rampUpPos);
                    }
                    if (timer.seconds() > 1.5) {
                        robot.shooterMotor.setPower(0.0);
                        currentTargetIndex = (currentTargetIndex + 1) % targetSequence.length;
                        robot.pushRampServo.setPosition(RobotActionConfig.rampResetPos);
                        state = OFFTAKEBALLSTATE.OFFTAKE_EJECTING;
                        timer.reset();
                    }
                }
                break;

            case OFFTAKE_EJECTING:
                // add ball removing from list here
                ejectCurrentBall(targetBall);
                if (timer.seconds() > 0.2) {
                    state = OFFTAKEBALLSTATE.OFFTAKE_AIMING; // Continue with next ball if any
                }
                break;

            case OFFTAKE_DONE:
                // Sequence complete or no matching ball
                if (gamepad2.getButton(GamepadKeys.Button.B)) {
                    state = OFFTAKEBALLSTATE.OFFTAKE_IDLE;
                }
                break;
        }
    }
    /// =====================================================================================
    // --- find the next ball that matches color sequence ---
    private Ball findNextTargetBall() {
        if (currentTargetIndex >= targetSequence.length) {
            return null; // No more balls to shoot
        };
        String targetColor = targetSequence[currentTargetIndex];
        for (Ball b : balls) {
            if (b.hasBall() && b.getBallColor().equalsIgnoreCase(targetColor)) {
                return b;
            }
        }
        // No ball of this color found
        return null;
    }

    //---- update - eject ball from shared ball list -----
    private void ejectCurrentBall(Ball b) {
        if (b != null) {
            b.setHasBall(false);
            b.setBallColor("Empty");
        }
    }

    // --- check if the sequence is complete ---
    public boolean isSortingComplete(){return state == OFFTAKEBALLSTATE.OFFTAKE_DONE;}
    // --- getters ---
    public OFFTAKEBALLSTATE getState() { return state; }
    // ---- set sequence ------
    public void setSequence(String[] sequence) { this.targetSequence = sequence; }
    // --- setters ---
    public void setState(OFFTAKEBALLSTATE state) { this.state = state; }

}
