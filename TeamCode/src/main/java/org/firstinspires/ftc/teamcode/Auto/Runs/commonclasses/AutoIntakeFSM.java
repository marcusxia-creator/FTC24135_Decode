package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AutoBallColors;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AutoColorDetection;

import org.firstinspires.ftc.teamcode.TeleOps.FSMIntake;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

@Config
public class AutoIntakeFSM {
    private final RobotHardware robot;

    public AutoIntakeFSM(RobotHardware robot) {
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
        private final AutoColorDetection colorDetection;
        private AutoBallColors ballColors;

        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime intakeTimer = new ElapsedTime();
        private final double maxRunTime;

        private INTAKESTATE currentState;

        private int targetSlot = 0;

        /// Constructor
        public IntakeRunMode(RobotHardware robot, double maxRunTime) {
            this.robot = robot;
            this.currentState = INTAKESTATE.INTAKE_INIT;
            this.colorDetection = new AutoColorDetection(robot);
            this.ballColors = AutoBallColors.UNKNOWN;
            this.maxRunTime = maxRunTime;
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
                    colorDetection.detectInit();
                    intakeTimer.reset();
                    SpindexerRunTo(0);
                    currentState = INTAKESTATE.INTAKE_RUN;
                    break;
                case INTAKE_RUN:
                    robot.kickerServo.setPosition(kickerRetract);
                    robot.intakeMotor.setPower(0.75);
                    stateTimer.reset();
                    currentState = INTAKESTATE.INTAKE_DETECT;
                    break;
                case INTAKE_DETECT:
                    if (colorDetection.isBallPresent()) {
                        targetSlot++;
                        stateTimer.reset();
                        currentState = INTAKESTATE.INTAKE_PAUSE;
                    } else if (intakeTimer.seconds() > maxRunTime) {
                        currentState = INTAKESTATE.INTAKE_END;
                    } else {
                        currentState = INTAKESTATE.INTAKE_RUN;
                    }
                    break;
                case INTAKE_PAUSE:
                    robot.intakeMotor.setPower(0.45);
                    if (stateTimer.seconds() > 0.2) {
                        stateTimer.reset();
                        currentState = INTAKESTATE.INTAKE_INDEX;
                    }
                    break;
                case INTAKE_INDEX:
                    if (targetSlot < 3) {
                        SpindexerRunTo(targetSlot);
                        stateTimer.reset();
                        currentState = INTAKESTATE.INTAKE_SPIN;
                    } else {
                        stateTimer.reset();
                        currentState = INTAKESTATE.INTAKE_UNJAM;
                    }
                    break;
                case INTAKE_SPIN:
                    if (stateTimer.seconds() > 0.4) {
                        currentState = INTAKESTATE.INTAKE_RUN;
                    }
                    break;
                case INTAKE_UNJAM:
                    SpindexerRunTo(0);
                    robot.intakeMotor.setPower(ejectSpeed);
                    if (stateTimer.seconds() > 0.5) {
                        currentState = INTAKESTATE.INTAKE_END;
                    }
                    break;
                case INTAKE_END:
                    robot.intakeMotor.setPower(intakeStop);
                    break;

            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            FSMIntakeRun();
            telemetryPacket.put("FSM Intake State", currentState);
            return currentState != INTAKESTATE.INTAKE_END;
        }
    }

    public Action IntakeRun (double maxTime) {
        return new IntakeRunMode(robot, maxTime);
    }
}
