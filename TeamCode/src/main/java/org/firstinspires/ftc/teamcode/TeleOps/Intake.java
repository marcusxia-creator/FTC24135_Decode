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

    float[] hsvValues = {0F,0F,0F};

    int[] none = {105, 110};

    int[] greenRangeLow = {120, 130};
    int[] greenRangeHigh = {145, 165};

    int[] purpleRangeLow = {115, 118};
    int[] purpleRangeHigh = {180, 230};

    boolean[] isFilled = {false, false, false};

    private int slotNumber = 1;

    private enum IntakeStates {
        INTAKE_INIT,
        INTAKE_START,
        DETECT_COLOR,
        SWITCH_SLOT,
        INTAKE_STOP
    }

    private IntakeStates intakeStates = IntakeStates.INTAKE_INIT;

    private ElapsedTime debounceTimer = new ElapsedTime();
    private double DEBOUNCE_THRESHOLD = 0.25;

    private ElapsedTime intakeTimer = new ElapsedTime();

    private final RobotHardware robot;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;


    public Intake(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }

    public void loop() {
        switch (intakeStates) {
            case INTAKE_INIT:
                robot.intakeMotor.setPower(0);
                robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);
                if (gamepad_1.getButton(GamepadKeys.Button.A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    robot.intakeMotor.setPower(RobotActionConfig.intakeSpeed);
                    intakeStates = IntakeStates.INTAKE_START;
                }
                break;
            case INTAKE_START:
                robot.intakeMotor.setPower(RobotActionConfig.intakeSpeed);
                double distance = robot.distanceSensor.getDistance(DistanceUnit.CM);
                if (distance < 10) {
                    intakeTimer.reset();
                    intakeStates = IntakeStates.DETECT_COLOR;
                }
                break;
            case DETECT_COLOR:
                Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);
                robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
                robot.rightGateServo.setPosition(RobotActionConfig.gateDown);
                robot.intakeMotor. setPower(0);
                isFilled[slotNumber - 1] = true;

                /**
                if ((hsvValues[0] > greenRangeHigh[0] && hsvValues[0] < greenRangeHigh[1]) || (hsvValues[0] > greenRangeLow[0] && hsvValues[0] < greenRangeLow[1])) {
                    artifacts.putArtifacts(slotNumber, 1);
                }

                if ((hsvValues[0] > purpleRangeHigh[0] && hsvValues[0] < purpleRangeHigh[1]) || (hsvValues[0] > purpleRangeLow[0] && hsvValues[0] < purpleRangeLow[1])) {
                    artifacts.putArtifacts(slotNumber, 2);
                }

                if (intakeTimer.seconds() > 0.3) {
                    artifacts.putArtifacts(slotNumber, 3);
                    intakeStates = IntakeStates.SWITCH_SLOT;
                }
                 */

                break;
            case SWITCH_SLOT:
                if (slotNumber == 1) {
                    robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot2);
                    slotNumber = 2;
                    intakeStates = IntakeStates.INTAKE_START;
                }
                if (slotNumber == 2) {
                    robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);
                    slotNumber = 3;
                    intakeStates = IntakeStates.INTAKE_START;
                }
                if (slotNumber == 3) {
                    slotNumber = 1;
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_STOP;
                }
                break;
            case INTAKE_STOP:
                if (intakeTimer.seconds() < 0.2) {
                    robot.intakeMotor.setPower(-0.2);
                }
                if (isFilled[0] && isFilled[1] && isFilled[2]) {
                    intakeStates = IntakeStates.INTAKE_INIT;
                }
                break;
            default:
                intakeStates = IntakeStates.INTAKE_INIT;
        }
    }
}
