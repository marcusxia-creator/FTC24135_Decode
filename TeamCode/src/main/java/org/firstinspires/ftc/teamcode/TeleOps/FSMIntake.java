package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

        INTAKE_STOP
    }

    public IntakeStates intakeStates = IntakeStates.INTAKE_IDLE;

    private ElapsedTime debounceTimer = new ElapsedTime();
    private double DEBOUNCE_THRESHOLD = 0.25;

    private ElapsedTime intakeTimer = new ElapsedTime();

    private final RobotHardware robot;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;

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
            case INTAKE_IDLE:
                if (gamepadManager.IntakeRun.PressState && spindexer.checkFor(Spindexer.SLOT.Empty)) {
                    intakeStates = IntakeStates.INTAKE_START;
                }
                break;

            case INTAKE_START:
                robot.leftGateServo.setPosition(gateUp);
                robot.rightGateServo.setPosition(gateUp);
                robot.intakeMotor.setPower(intakeSpeed);
                if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 10) {
                    recorded = false;
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_CAPTURE;
                }
                break;

            case INTAKE_CAPTURE:

                robot.intakeMotor.setPower(intakeSpeed);
                //Put gates down
                robot.leftGateServo.setPosition(gateDown);
                robot.rightGateServo.setPosition(gateDown);
                robot.intakeMotor.setPower(intakeSpeed);

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
        }

        if (gamepadManager.IntakeRun.PressState||!spindexer.checkFor(Spindexer.SLOT.Empty)){
            intakeStates=IntakeStates.INTAKE_STOP;
        }

        if (gamepadManager.IntakeReverse.PressState){
            robot.intakeMotor.setPower(ejectSpeed);

            if (gamepadManager.IntakeReverse.PressState && isButtonDebounced()) {
                intakeStates=IntakeStates.INTAKE_STOP;
            }
        }
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
}
