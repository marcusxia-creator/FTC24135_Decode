package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.BallColors;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.ColorDetection;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;
public class Intake {
    private final RobotHardware robot;

    public Intake (RobotHardware robot) {
        this.robot = robot;
    }

    public static class IntakeRunMode implements Action {
        public enum INTAKESTATE {
            INTAKE_INIT,
            INTAKE_RUN,
            INTAKE_DETECT,
            INTAKE_PAUSE,
            INTAKE_INDEX,
            INTAKE_SPIN,
            INTAKE_UNJAM,
            INTAKE_END

        }

        /// Variables
        private RobotHardware robot;
        public ColorDetection colorDetection;

        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime intakeTimer = new ElapsedTime();

        private INTAKESTATE currentState;
        private int targetSlot = 0;
        private BallColors currentColor;
        public int greenSlot;
        public int shootingInitSlot;

        /// Constructor
        public IntakeRunMode(RobotHardware robot) {
            this.robot = robot;
            this.currentState = INTAKESTATE.INTAKE_INIT;
        }

        public void SpindexerRunTo(int slot) {
            if (slot == 0) {
                robot.spindexerServo.setPosition(spindexerSlot1);
            }
            if (slot == 1) {
                robot.spindexerServo.setPosition(spindexerSlot2);
            }
            if (slot == 2) {
                robot.spindexerServo.setPosition(spindexerSlot3);
            }
        }

        public void FSMIntakeRun() {
            colorDetection.updateDetection();
            switch (currentState) {
                case INTAKE_INIT:
                    greenSlot = -1;
                    colorDetection.detectInit();
                    intakeTimer.reset();
                    SpindexerRunTo(0);
                    currentState = INTAKESTATE.INTAKE_RUN;
                    break;
                case INTAKE_RUN:
                    robot.intakeMotor.setPower(0.95);
                    stateTimer.reset();
                    currentState = INTAKESTATE.INTAKE_DETECT;
                    break;
                case INTAKE_DETECT:
                    if (colorDetection.isBallPresent()) {
                        if (colorDetection.getColor() == BallColors.GREEN) {
                            greenSlot = targetSlot + 1;
                            stateTimer.reset();
                            currentState = INTAKESTATE.INTAKE_PAUSE;
                        } else {
                            stateTimer.reset();
                            currentState = INTAKESTATE.INTAKE_PAUSE;
                        }
                    } else if (intakeTimer.seconds() > 12) {
                        currentState = INTAKESTATE.INTAKE_END;
                    } else {
                        currentState = INTAKESTATE.INTAKE_RUN;
                    }
                    break;
                case INTAKE_PAUSE:
                    robot.intakeMotor.setPower(intakeStop);
                    if (stateTimer.seconds() > 0.2) {
                        targetSlot++;
                        currentState = INTAKESTATE.INTAKE_INDEX;
                    }
                    break;
                case INTAKE_INDEX:
                    if (targetSlot <= 2) {
                        SpindexerRunTo(targetSlot);
                        stateTimer.reset();
                        currentState = INTAKESTATE.INTAKE_SPIN;
                    } else if (targetSlot >= 3) {
                        currentState = INTAKESTATE.INTAKE_END;
                    }
                    break;
                case INTAKE_SPIN:
                    if (stateTimer.seconds() > 0.2) {
                        currentState = INTAKESTATE.INTAKE_RUN;
                    }
                    break;
                case INTAKE_UNJAM:
                    robot.intakeMotor.setPower(ejectSpeed);
                    currentState = INTAKESTATE.INTAKE_RUN;
                    break;
                case INTAKE_END:
                    robot.intakeMotor.setPower(intakeStop);
                    break;

            }
        }

        private void updateShootingInitSlot() {
            if (greenSlot == -1) {
                shootingInitSlot = 2;
            } else {
                shootingInitSlot = Math.floorMod(greenSlot - targetGreenSlot, 3) + 1;
            }
        }

        public int getShootingInitSlot() {
            return shootingInitSlot;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.put("FSM Intake State", currentState);
            FSMIntakeRun();
            updateShootingInitSlot();
            return currentState != INTAKESTATE.INTAKE_END;
        }
    }

    public Action IntakeRun () {
        return new IntakeRunMode(robot);
    }
}
