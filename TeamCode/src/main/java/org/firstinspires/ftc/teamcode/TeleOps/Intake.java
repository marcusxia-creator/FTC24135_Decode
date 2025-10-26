package org.firstinspires.ftc.teamcode.TeleOps;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    public ColorSensor colorSensor;
    private final HardwareMap hardwareMap;

    float[] hsvValues = {0F,0F,0F};

    int[] none = {105, 110};

    int[] greenRangeLow = {120, 130};
    int[] greenRangeHigh = {145, 165};

    int[] purpleRangeLow = {115, 118};
    int[] purpleRangeHigh = {180, 230};

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


    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
    }

    public void loop() {
        switch (intakeStates) {
            case INTAKE_INIT:
                robot.intakeMotor.setPower(0);
                robot.indexServo.setPosition(RobotActionConfig.indexServoPosition1);
                artifacts.init();
                if (gamepad1.a && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    robot.intakeMotor.setPower(RobotActionConfig.intake_Motor_Power);
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_START;
                }
                break;
            case INTAKE_START:
                robot.intakeMotor.setPower(RobotActionConfig.intake_Motor_Power);
                if (intakeTimer.seconds() > 0.3) {
                    intakeTimer.reset();
                    intakeStates = IntakeStates.DETECT_COLOR;
                }
                break;
            case DETECT_COLOR:
                Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                robot.intakeMotor.setPower(RobotActionConfig.intake_Motor_Power);

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
                break;
            case SWITCH_SLOT:
                robot.intakeMotor.setPower(RobotActionConfig.intake_Motor_Power);
                if (slotNumber == 1) {
                    robot.indexServo.setPosition(RobotActionConfig.indexServoPosition2);
                    slotNumber = 2;
                    intakeStates = IntakeStates.INTAKE_START;
                }
                if (slotNumber == 2) {
                    robot.indexServo.setPosition(RobotActionConfig.indexServoPosition3);
                    slotNumber = 3;
                    intakeStates = IntakeStates.INTAKE_START;
                }
                if (slotNumber == 3) {
                    slotNumber = 1;
                    intakeStates = IntakeStates.INTAKE_STOP;
                }
                break;
            case INTAKE_STOP:
                robot.intakeMotor.setPower(0);
                if (depositStates == DePositStates.DEPOSIT_STOP) {
                    intakeStates = IntakeStates.INTAKE_INIT;
                }
                break;
            default:
                intakeStates = IntakeStates.INTAKE_INIT;
        }
    }
}
