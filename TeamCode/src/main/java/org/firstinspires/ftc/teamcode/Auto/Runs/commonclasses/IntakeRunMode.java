package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.ejectSpeed;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.gateDown;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.gateUp;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.intakeStop;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.spindexerSlot0;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.spindexerSlot1;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.spindexerSlot2;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class IntakeRunMode implements Action {

    public enum INTAKESTATE {
        INTAKE_INIT,
        INTAKE_READY,
        INTAKE_RUN,
        INTAKE_DETECT,
        INTAKE_PAUSE,
        INTAKE_INDEX,
        INTAKE_SPIN,
        INTAKE_UNJAM,
        INTAKE_END

    }

    /// Variables
    private final RobotHardware robot;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime colorSensorTimer = new ElapsedTime();
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private INTAKESTATE currentState = INTAKESTATE.INTAKE_READY;
    private int targetSlot = 0;

    /// Constructor
    public IntakeRunMode(RobotHardware robot) {
        this.robot = robot;
        this.currentState = INTAKESTATE.INTAKE_READY;
    }

    public void SpindexerRunTo(int slot){
        if(slot==0){
            robot.spindexerServo.setPosition(spindexerSlot0);
        }
        if(slot==1){
            robot.spindexerServo.setPosition(spindexerSlot1);
        }
        if(slot==2){
            robot.spindexerServo.setPosition(spindexerSlot2);
        }
    }

    public void FSMIntakeRun() {
        switch (currentState) {
            case INTAKE_INIT:
                SpindexerRunTo(0);
                robot.leftGateServo.setPosition(gateUp);
                robot.rightGateServo.setPosition(gateUp);
                currentState = INTAKESTATE.INTAKE_READY;
                break;
            case INTAKE_READY:
                robot.leftGateServo.setPosition(gateUp);
                robot.rightGateServo.setPosition(gateUp);
                currentState = INTAKESTATE.INTAKE_RUN;
                break;
            case INTAKE_RUN:
                robot.intakeMotor.setPower(0.8);
                currentState = INTAKESTATE.INTAKE_DETECT;
                break;
            case INTAKE_DETECT:
                if (robot.distanceSensor.getDistance(DistanceUnit.MM) < 50) {
                    stateTimer.reset();
                    currentState = INTAKESTATE.INTAKE_PAUSE;
                } else {
                    currentState = INTAKESTATE.INTAKE_RUN;
                }
                break;
            case INTAKE_PAUSE:
                robot.intakeMotor.setPower(intakeStop);
                robot.leftGateServo.setPosition(gateDown);
                robot.rightGateServo.setPosition(gateDown);
                if (stateTimer.seconds()>0.2) {
                    targetSlot++;
                    currentState = INTAKESTATE.INTAKE_INDEX;
                }
                break;
            case INTAKE_INDEX:
                if (targetSlot <= 2){
                    //need to set spindexer to spin here.
                    SpindexerRunTo(targetSlot);
                    stateTimer.reset();
                    currentState = INTAKESTATE.INTAKE_SPIN;
                }else if(targetSlot >= 3){
                    currentState = INTAKESTATE.INTAKE_END;
                }
                break;
            case INTAKE_SPIN:
                if (stateTimer.seconds()>0.2){
                    currentState = INTAKESTATE.INTAKE_READY;
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

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        telemetryPacket.put("FSM Intake State", currentState);
        FSMIntakeRun();
        return currentState != INTAKESTATE.INTAKE_END;
    }
}
