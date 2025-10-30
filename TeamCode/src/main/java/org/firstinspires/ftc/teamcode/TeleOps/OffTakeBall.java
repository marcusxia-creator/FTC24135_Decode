package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private final List<Ball> balls;
    private List<Ball> ballsWithBall;
    private Ball targetBall;

    // --- Color sequence logic (now enum-based) ---
    private BallColor[] targetSequence = {
            BallColor.PURPLE,  // adjust depending on your defined colors
            BallColor.GREEN,
            BallColor.PURPLE
    };
    private boolean useColorSequence = true;
    private int currentColorTargetIndex = 0;    // for color based sequence indexing
    private int currentCounterIndex = 0;   // for non color based sequence indexing
    private int cycle_no = 0;      // counter for non color based sequence times

    // --- Constructor ---
    public OffTakeBall(RobotHardware robot, GamepadEx gamepad2, List<Ball> balls) {
        this.robot = robot;
        this.gamepad2 = gamepad2;
        this.balls = balls;
    }

    // --- Main update loop ---
    public void update() {
        switch (state) {

            case OFFTAKE_IDLE:
                robot.shooterMotor.setPower(0.0);

                if (gamepad2.getButton(GamepadKeys.Button.Y)) {
                    // Prepare ball list
                    ballsWithBall = balls.stream()
                            .filter(Ball::hasBall)
                            .collect(Collectors.toList());

                    cycle_no = ballsWithBall.size();
                    currentColorTargetIndex = 0;
                    currentCounterIndex = 0;
                    timer.reset();

                    // Determine mode for this cycle
                    useColorSequence = !isUnknownSequence(targetSequence);
                    state = OFFTAKEBALLSTATE.OFFTAKE_AIMING;
                }
                break;

            case OFFTAKE_AIMING:
                targetBall = getNextTargetBall();

                if (targetBall != null) {
                    robot.spindexerServo.setPosition(targetBall.getSlotAngle());
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
// === Helper methods ===
// ===============================================================

    /** Selects the next target ball based on current mode. */
    private Ball getNextTargetBall() {
        if (useColorSequence) {
            // Tag-guided mode
            if (currentColorTargetIndex < targetSequence.length) {
                return findNextTargetBall(currentColorTargetIndex);
            }
        } else {
            // Sequential (free shooting) mode
            if (currentCounterIndex < cycle_no) {
                return ballsWithBall.get(cycle_no - currentCounterIndex - 1);
            }
        }
        return null;
    }

    /** Handles spin-up, fire, and transition to ejecting. */
    private void handleShootingState() {
        robot.shooterMotor.setPower(SHOOTER_POWER);

        if (timer.seconds() > RAMP_UP_TIME) {
            robot.leftGateServo.setPosition(GATEUP);
            robot.rightGateServo.setPosition(GATEUP);
            robot.pushRampServo.setPosition(RAMP_UP);
        }

        if (timer.seconds() > FIRE_TIME) {
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
        ejectCurrentBall(targetBall);

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
    /** --- Find next ball matching target sequence color ---*/
    private Ball findNextTargetBall(int targetIndex) {
        if (!isUnknownSequence(targetSequence)) {
            if (targetIndex >= targetSequence.length) {
                return null; // all done
            }
            BallColor targetColor = targetSequence[targetIndex];
            for (Ball b : balls) {
                if (b.hasBall() && b.getColor() == targetColor) {
                    return b;
                }
            }
            return null; // none found
        }
        return null;
    }

    // --- Eject current ball (mark slot empty) ---
    private void ejectCurrentBall(Ball b) {
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
}
