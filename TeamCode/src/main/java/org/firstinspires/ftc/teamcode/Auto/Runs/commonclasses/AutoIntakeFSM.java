package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AutoColorDetection;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

@Config
public class AutoIntakeFSM {
    private final RobotHardware robot;
    private final AutoSpindexerContext spindexerContext;

    public AutoIntakeFSM(RobotHardware robot, AutoSpindexerContext context) {
        this.spindexerContext = context;
        this.robot = robot;
    }

    public static class IntakeRunMode implements Action {
        public enum INTAKESTATE {
            INTAKE_INIT,
            INTAKE_PREP,
            INTAKE_RUN,
            INTAKE_DETECT,
            INTAKE_END

        }

        /// Variables
        private RobotHardware robot;
        private AutoSpindexerContext spindexerContext;
        private final AutoColorDetection colorDetection;

        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime intakeTimer = new ElapsedTime();
        private final double maxRunTime;

        private INTAKESTATE currentState;

        /// Constructor
        public IntakeRunMode(RobotHardware robot, double maxRunTime, AutoSpindexerContext spindexerContext) {
            this.robot = robot;
            this.spindexerContext = spindexerContext;
            this.currentState = INTAKESTATE.INTAKE_INIT;
            this.colorDetection = new AutoColorDetection(robot);
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
            switch (currentState) {
                case INTAKE_INIT:
                    colorDetection.detectInit();
                    intakeTimer.reset();
                    stateTimer.reset();
                    robot.kickerServo.setPosition(kickerRetract);
                    currentState = INTAKESTATE.INTAKE_PREP;
                    break;
                case INTAKE_PREP:
                    if (stateTimer.seconds() > 0.1) {
                        SpindexerRunTo(1);
                        if (stateTimer.seconds()>0.5){
                            stateTimer.reset();
                            currentState = INTAKESTATE.INTAKE_RUN;
                        }
                    }
                    break;
                case INTAKE_RUN:
                    colorDetection.updateSlotColors();
                    robot.intakeMotor.setPower(0.95);
                    stateTimer.reset();
                    currentState = INTAKESTATE.INTAKE_DETECT;
                    break;
                case INTAKE_DETECT:
                    if (colorDetection.isSpindexerFull()) {
                        colorDetection.updateSlotColors();
                        spindexerContext.currentGreenSlot = colorDetection.findGreenSlot();
                        spindexerContext.updateShootingInitSlot();
                        robot.intakeMotor.setPower(0);
                        if (stateTimer.seconds()>0.2) {
                            stateTimer.reset();
                            currentState = INTAKESTATE.INTAKE_END;
                        }
                    } else if (intakeTimer.seconds() > maxRunTime) {
                        colorDetection.updateSlotColors();
                        spindexerContext.currentGreenSlot = colorDetection.findGreenSlot();
                        spindexerContext.updateShootingInitSlot();
                        robot.intakeMotor.setPower(0);
                        if (stateTimer.seconds()>0.2) {
                            stateTimer.reset();
                            currentState = INTAKESTATE.INTAKE_END;
                        }
                    } else {
                        currentState = INTAKESTATE.INTAKE_RUN;
                    }
                    break;
                case INTAKE_END:
                    break;

            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            FSMIntakeRun();
            telemetryPacket.put("FSM Intake State", currentState);
            telemetryPacket.put("Is Spindexer Full",colorDetection.isSpindexerFull());
            telemetryPacket.put("Calc Shooting Int Slot", spindexerContext.shootingInitSlot);
            telemetryPacket.put("Detected Green Slot", colorDetection.findGreenSlot());
            telemetryPacket.put("Transferred Green Slot",spindexerContext.currentGreenSlot);
            return currentState != INTAKESTATE.INTAKE_END;
        }
    }

    public Action IntakeRun (double maxTime) {
        return new IntakeRunMode(robot, maxTime, spindexerContext);
    }
}
