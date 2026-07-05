package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import java.util.Collections;
import java.util.stream.Collectors;

public class FSMIntake {

    /**
     * Color Range:
     * None: 106 - 110
     * Purple: 115 - 125, 200 - 230
     * Green: 120 - 130, 145 - 160
     */

    public enum IntakeStates {
        INTAKE_IDLE,
        INTAKE_PREP,
        INTAKE_START,
        INTAKE_CAPTURE,
        INTAKE_STOP,
        INTAKE_REVERSE,
    }

    public static IntakeStates intakeStates = IntakeStates.INTAKE_IDLE;

    private ElapsedTime unjamTimer = new ElapsedTime();
    private ElapsedTime jammedTimer = new ElapsedTime();

    public static ElapsedTime intakeTimer = new ElapsedTime();

    private final RobotHardware robot;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private double intakeRPM;
    private boolean reversing = false;
    private boolean stopMoveRequested = false;

    boolean recorded;

    public FSMIntake(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }

    public void loop() {
        switch (intakeStates) {
            case INTAKE_IDLE:
                reversing = false;
                robot.intakeMotor.setPower(0);
                break;

            case INTAKE_PREP:
                // Ensure we start at a clean position
                robot.spindexerServo.setPosition(spindexerIntakePos);
                intakeTimer.reset();
                intakeStates = IntakeStates.INTAKE_START;
                break;

            case INTAKE_START:
                boolean jammed = isIntakeJammmed();
                HandleIntaking(jammed); // This manages motor power internally
                intakeTimer.reset();
                //go straight to capture, might fuse states
                intakeStates = IntakeStates.INTAKE_CAPTURE;
                break;

            case INTAKE_CAPTURE:
                // This conditions previously cleared vote buffers, might be needed if colour voting is used
                if (intakeTimer.seconds() < 0.05) {}
                //nned to make this longer
                else if (intakeTimer.seconds() < 0.10) {// Collect as many samples as possible in 200ms
                    }
                // count the number of full slots. if all three slots are full, stop
                if (Collections.frequency(robot.slotSensors.stream().map(SlotSensor::checkBall).collect(Collectors.toList()), Boolean.TRUE)>=requiredSensors) {
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_STOP;
                }
                break;


            case INTAKE_STOP:
                robot.intakeMotor.setPower(ejectSpeed);
                double time = intakeTimer.seconds();
                //Park spindexer at travel pos
                robot.spindexerServo.setPosition(spindexerStowPos);
                //Might need delay here
                intakeStates = IntakeStates.INTAKE_IDLE;
                break;

            case INTAKE_REVERSE:
                if (intakeTimer.seconds() < 0.5) {
                    robot.intakeMotor.setPower(ejectSpeed);
                }else{
                    intakeStates = IntakeStates.INTAKE_IDLE;
                }
                break;
            default:
                intakeStates = IntakeStates.INTAKE_IDLE;
                break;
        }
    }


    private boolean isIntakeJammmed() {
        double intakeTicksPerSecond = robot.intakeMotor.getVelocity();
        intakeRPM = intakeTicksPerSecond * INTAKE_RPM_CONVERSION;
        if (robot.intakeMotor.getPower() > 0.2 && intakeRPM < 400) {
            if (jammedTimer.seconds() > 0.2) {
                return true;
            }
        }
        return false;
    }
    public void reversing (){
        reversing = true;
        unjamTimer.reset();
        robot.intakeMotor.setPower(-0.4);
    }
    private void HandleIntaking (boolean jammed) {
        if (jammed && !reversing){
            reversing = true;
            unjamTimer.reset();
        }
        if (reversing){
            if (unjamTimer.seconds() > 0.25) {
                robot.intakeMotor.setPower(-0.5);
            }
            if (unjamTimer.seconds() > 0.5){
                robot.intakeMotor.setPower(0.0);
                reversing = false;
            }
        }
        if (!jammed && !reversing) {
            robot.intakeMotor.setPower(intakeSpeed);
        }
    }

    /**
     * Added New for safe change of state
     */
    public boolean canExit() {
        return intakeStates == IntakeStates.INTAKE_IDLE;
    }

    public void requestGracefulStop() {
        // Only request STOP if we are not already stopping/idle
        if (intakeStates != IntakeStates.INTAKE_IDLE && intakeStates != IntakeStates.INTAKE_STOP) {
            intakeTimer.reset();
            intakeStates = IntakeStates.INTAKE_STOP;
        }
    }
}