package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class FSMIntake {

    /**
     * Color Range:
     * None: 106 - 110
     * Purple: 115 - 125, 200 - 230
     * Green: 120 - 130, 145 - 160
     */

    private enum IntakeStates {
        INTAKE_IDLE,

        INTAKE_START,
        INTAKE_CAPTURE,

        INTAKE_STOP,
        INTAKE_UNJAM
    }

    public IntakeStates intakeStates = IntakeStates.INTAKE_IDLE;

    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime unjamTimer = new ElapsedTime();
    private ElapsedTime jammedTimer = new ElapsedTime();
    private double DEBOUNCE_THRESHOLD = 0.25;

    private ElapsedTime intakeTimer = new ElapsedTime();

    private final RobotHardware robot;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private double intakeRPM;
    private boolean reversing = false;


    Spindexer spindexer;
    GamepadManager gamepadManager;

    boolean recorded;

    public FSMIntake(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, Spindexer spindexer, GamepadManager gamepadManager) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;

        this.spindexer = spindexer;
        this.gamepadManager = gamepadManager;
    }

    public void loop() {
        switch (intakeStates) {
            //start of intake FSM
            case INTAKE_IDLE:
            if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
                intakeStates = IntakeStates.INTAKE_START;
            }
                break;
            //start intake motor
            case INTAKE_START:
                boolean jammed = isIntakeJammmed();
                robot.intakeMotor.setPower(intakeSpeed);
                robot.leftGateServo.setPosition(gateUp);
                robot.rightGateServo.setPosition(gateUp);
                HandleIntaking(jammed);

                if (robot.distanceSensor.getDistance(DistanceUnit.MM) < distanceThreshold) {
                    recorded = false;
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_CAPTURE;
                }

                break;
            //ball goes into spindxer
            case INTAKE_CAPTURE:
                robot.intakeMotor.setPower(intakeSpeed);
                //Put gates down
                robot.leftGateServo.setPosition(gateDown);
                robot.rightGateServo.setPosition(gateDown);
                if (intakeTimer.seconds() > gateDownTime && !recorded) {
                    spindexer.writeToCurrent(robot.colorSensor, robot.distanceSensor);
                    spindexer.runToSlot(Spindexer.SLOT.Empty);
                    recorded = true;
                }

                if (intakeTimer.seconds() > SpindexerMoveTime) {
                    if (spindexer.checkFor(Spindexer.SLOT.Empty)) {
                        intakeStates = IntakeStates.INTAKE_START;
                    } else {
                        intakeStates = IntakeStates.INTAKE_STOP;
                    }
                }
                break;

            case INTAKE_STOP:
                robot.intakeMotor.setPower(0);
                robot.shooterMotor.setPower(0);
                robot.leftGateServo.setPosition(gateDown);
                robot.rightGateServo.setPosition(gateDown);
                robot.pushRampServo.setPosition(rampDownPos);
                intakeStates = IntakeStates.INTAKE_IDLE;
                break;
            /*case INTAKE_UNJAM:
                if (unjamTimer.seconds() > 0.1) {
                    robot.intakeMotor.setPower(-0.5);
                } else if (unjamTimer.seconds() > 0.5) {
                    robot.intakeMotor.setPower(0);
                }
                if (unjamTimer.seconds() >= 0.5 && unjamTimer.seconds() < 0.75){
                    spindexer.runToSlot(spindexer.prevSlot);
                }
                if (unjamTimer.seconds() > 0.75){
                    robot.intakeMotor.setPower(intakeSpeed);
                    unjamTimer.reset();
                    intakeStates = IntakeStates.INTAKE_START;
                }
                break;

             */
        }

        if (gamepadManager.IntakeRun.PressState && isButtonDebounced()) {
            if (intakeStates == IntakeStates.INTAKE_START || intakeStates == IntakeStates.INTAKE_CAPTURE) {
                intakeStates = IntakeStates.INTAKE_STOP;
            }
        }
//        if (intakeStates == IntakeStates.INTAKE_START && !spindexer.checkFor(Spindexer.SLOT.Empty)){
//            intakeStates = IntakeStates.INTAKE_STOP;
//        }
//
//        if (gamepadManager.IntakeReverse.PressState) {
//            reversing = false;
//            robot.intakeMotor.setPower(ejectSpeed);
//
//            if (gamepadManager.IntakeReverse.PressState && isButtonDebounced()) {
//                intakeStates = IntakeStates.INTAKE_STOP;
//            }
//        }
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    private boolean isIntakeJammmed() {
        double intakeTicksPerSecond = robot.intakeMotor.getVelocity();
        intakeRPM = intakeTicksPerSecond * INTAKE_RPM_CONVERSION;
        if (robot.intakeMotor.getPower() > 0.2 && intakeRPM < 100) {
            if (jammedTimer.seconds() > 0.2) {
                return true;
            }
        }
        return false;
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
}