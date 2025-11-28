package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class ShooterRunMode implements Action {
    public enum SHOOTERSTATE {
        SHOOTER_INIT,
        SHOOTER_RUN,
        SHOOTER_SWITCH,
        SHOOTER_LAUNCH,
        SHOOTER_RESET_1,
        SHOOTER_RESET_2,
        SHOOTER_END
    }

    /// Variables
    private final RobotHardware robot;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime stateTimer2 = new ElapsedTime();
    private final ElapsedTime colorSensorTimer = new ElapsedTime();
    private int targetSlot = 2;
    private double ShooterWaitTime;
    private double ShotPower;
    private SHOOTERSTATE currentState;

    public ShooterRunMode(RobotHardware robot, double ShotPower, double ShooterWaitTime) {
        this.robot = robot;
        this.ShotPower = ShotPower;
        this.ShooterWaitTime = ShooterWaitTime;
        this.currentState = SHOOTERSTATE.SHOOTER_INIT;
    }

    public void SpindexerRunTo(int slot) {
        if (slot == 0) {
            robot.spindexerServo.setPosition(spindexerSlot0);
        }
        if (slot == 1) {
            robot.spindexerServo.setPosition(spindexerSlot1);
        }
        if (slot == 2) {
            robot.spindexerServo.setPosition(spindexerSlot2);
        }
    }

    public void FSMShooterRun() {
        switch (currentState) {
            case SHOOTER_INIT:
                robot.pushRampServo.setPosition(rampDownPos);
                robot.leftGateServo.setPosition(gateDown);
                robot.rightGateServo.setPosition(gateDown);
                stateTimer.reset();
                currentState = SHOOTERSTATE.SHOOTER_RUN;
                break;
            case SHOOTER_RUN:
                SpindexerRunTo(targetSlot);
                robot.shooterMotor.setPower(ShotPower);
                if (stateTimer.seconds() > ShooterWaitTime) {
                    stateTimer2.reset();
                    currentState = SHOOTERSTATE.SHOOTER_LAUNCH;
                }
                break;
            case SHOOTER_LAUNCH:
                robot.leftGateServo.setPosition(gateUp);
                robot.rightGateServo.setPosition(gateUp);
                if (stateTimer2.seconds() > 0.5) {
                    robot.pushRampServo.setPosition(rampUpPos);
                    stateTimer.reset();
                    currentState = SHOOTERSTATE.SHOOTER_RESET_1;
                }
                break;
            case SHOOTER_RESET_1:
                if (stateTimer.seconds() > 0.3) {
                    robot.pushRampServo.setPosition(rampDownPos);
                    stateTimer2.reset();
                    currentState = SHOOTERSTATE.SHOOTER_RESET_2;
                }
                break;
            case SHOOTER_RESET_2:
                if (stateTimer2.seconds() > 0.2) {
                    robot.leftGateServo.setPosition(gateDown);
                    robot.rightGateServo.setPosition(gateDown);
                    targetSlot--;
                    stateTimer.reset();
                    currentState = SHOOTERSTATE.SHOOTER_SWITCH;
                }
                break;
            case SHOOTER_SWITCH:
                if (targetSlot >= 0) {
                    //need to set spindexer to spin here.
                    SpindexerRunTo(targetSlot);
                    if (stateTimer.seconds() > 0.7) {
                        stateTimer2.reset();
                        currentState = SHOOTERSTATE.SHOOTER_LAUNCH;
                    }
                } else if (targetSlot < 0) {
                    currentState = SHOOTERSTATE.SHOOTER_END;
                }
                break;
            case SHOOTER_END:
                robot.shooterMotor.setPower(0);
                break;
        }
    }


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        telemetryPacket.put("FSM Intake State", currentState);
        FSMShooterRun();
        return currentState != SHOOTERSTATE.SHOOTER_END;
    }
}
