package org.firstinspires.ftc.teamcode.TeleOps;

import android.graphics.Color;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    /**
     * Color Range:
     * None: 106 - 110
     * Purple: 115 - 125, 200 - 230
     * Green: 120 - 130, 145 - 160
     */

    private enum IntakeStates {
        INTAKE_STBY,

        INTAKE_RUNNING,
        INTAKE_CAPTURE,

        INTAKE_REVERSE
    }

    public IntakeStates intakeStates = IntakeStates.INTAKE_STBY;

    private ElapsedTime debounceTimer = new ElapsedTime();
    private double DEBOUNCE_THRESHOLD = 0.25;

    private ElapsedTime intakeTimer = new ElapsedTime();

    private final RobotHardware robot;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;

    Spindexer spindexer;
    GamepadManager gamepadManager;

    boolean recorded;

    public Intake(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, Spindexer spindexer, GamepadManager gamepadManager) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;

        this.spindexer = spindexer;
        this.gamepadManager = gamepadManager;
    }

    public void loop() {
        switch (intakeStates) {
            case INTAKE_STBY:
                robot.intakeMotor.setPower(0);
                if (gamepadManager.IntakeRun.PressState && spindexer.checkFor(Spindexer.SLOT.Empty)) {
                    intakeStates = IntakeStates.INTAKE_RUNNING;
                }
                if (gamepadManager.IntakeReverse.PressState) {
                    intakeStates = IntakeStates.INTAKE_REVERSE;
                }
                break;

            case INTAKE_RUNNING:
                robot.intakeMotor.setPower(RobotActionConfig.intakeSpeed);
                robot.leftGateServo.setPosition(RobotActionConfig.gateUp);
                robot.rightGateServo.setPosition(RobotActionConfig.gateUp);
                if (robot.distanceSensor.getDistance(DistanceUnit.CM) < 10) {
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_CAPTURE;
                    recorded = false;
                }
                if (gamepadManager.IntakeRun.PressState||gamepadManager.IntakeReverse.PressState||!spindexer.checkFor(Spindexer.SLOT.Empty)){
                    intakeStates=IntakeStates.INTAKE_STBY;
                    robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
                    robot.rightGateServo.setPosition(RobotActionConfig.gateDown);
                }
                break;

            case INTAKE_CAPTURE:

                //Put gates down
                if (intakeTimer.seconds() > RobotActionConfig.gateDownTime) {
                    robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
                    robot.rightGateServo.setPosition(RobotActionConfig.gateDown);
                    //Run intake backward
                    //robot.intakeMotor.setPower(RobotActionConfig.ejectSpeed);
                    robot.intakeMotor.setPower(RobotActionConfig.intakeSpeed);
                }

                if (intakeTimer.seconds() > RobotActionConfig.SpindexerStartTime && !recorded) {
                    spindexer.writeToCurrent(robot.colorSensor, robot.distanceSensor);
                    spindexer.runToSlot(Spindexer.SLOT.Empty);
                    recorded = true;
                }

                if (intakeTimer.seconds() > RobotActionConfig.SpindexerMoveTime || robot.distanceSensor.getDistance(DistanceUnit.CM) > 10) {
                    if (spindexer.checkFor(Spindexer.SLOT.Empty)) {
                        robot.intakeMotor.setPower(RobotActionConfig.intakeSpeed);
                        intakeStates = IntakeStates.INTAKE_RUNNING;
                    } else {
                        intakeStates = IntakeStates.INTAKE_STBY;
                        robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
                        robot.rightGateServo.setPosition(RobotActionConfig.gateDown);
                        robot.intakeMotor.setPower(0);
                    }
                }
                break;

            case INTAKE_REVERSE:
                robot.intakeMotor.setPower(-RobotActionConfig.intakeSpeed);
                robot.leftGateServo.setPosition(RobotActionConfig.gateUp);
                robot.rightGateServo.setPosition(RobotActionConfig.gateUp);

                if (gamepadManager.IntakeRun.PressState||gamepadManager.IntakeReverse.PressState){
                    intakeStates=IntakeStates.INTAKE_STBY;
                    robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
                    robot.rightGateServo.setPosition(RobotActionConfig.gateDown);
                }
                break;
        }
        //STBY for spindexer manual control
        if(gamepadManager.Unjam.HoldState||gamepadManager.spinPrev.HoldState||gamepadManager.spinNext.HoldState){
            intakeStates = IntakeStates.INTAKE_STBY;
            robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
            robot.rightGateServo.setPosition(RobotActionConfig.gateDown);
            robot.intakeMotor.setPower(0);
        }
    }
}
