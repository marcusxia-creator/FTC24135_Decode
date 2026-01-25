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

    public enum IntakeStates {
        INTAKE_IDLE,
        INTAKE_PREP,
        INTAKE_START,
        INTAKE_CAPTURE,
        INTAKE_RUNTONEXT,
        INTAKE_STOP,
        INTAKE_REVERSE,
    }

    public IntakeStates intakeStates = IntakeStates.INTAKE_IDLE;

    private ElapsedTime unjamTimer = new ElapsedTime();
    private ElapsedTime jammedTimer = new ElapsedTime();

    public ElapsedTime intakeTimer = new ElapsedTime();

    private final RobotHardware robot;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private double intakeRPM;
    private boolean reversing = false;
    Spindexer spindexer;

    boolean recorded;

    public FSMIntake(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, Spindexer spindexer) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;

        this.spindexer = spindexer;
    }

    public void loop() {
        switch (intakeStates) {
            //start of intake FSM
            case INTAKE_IDLE:
                reversing = false;
                robot.intakeMotor.setPower(0);
                break;
            //start intake motor
            case INTAKE_PREP:
                spindexer.RuntoPosition(0);
                intakeTimer.reset();
                intakeStates = IntakeStates.INTAKE_START;
                break;
            case INTAKE_START:
                boolean jammed = isIntakeJammmed();
                if (!jammed) {
                    if (intakeTimer.seconds() > 0.1) {
                        robot.intakeMotor.setPower(intakeSpeed);
                    }
                }
                HandleIntaking(jammed);
                if (robot.distanceSensor.getDistance(DistanceUnit.MM) < distanceThreshold) {
                    //recorded = false;
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_CAPTURE;
                }
                break;
            //ball goes into spindxer
            case INTAKE_CAPTURE:
                if (intakeTimer.seconds()>0.1){
                    spindexer.writeToCurrent(robot.colorSensor, robot.distanceSensor);
                }
                if (intakeTimer.seconds() > 0.4) {
                    if (spindexer.checkFor(Spindexer.SLOT.Empty)) {
                        spindexer.RunToNext();
                        intakeStates = IntakeStates.INTAKE_RUNTONEXT;
                        intakeTimer.reset();
                    } else {
                        intakeStates = IntakeStates.INTAKE_STOP;
                        intakeTimer.reset();
                    }
                }
                break;
            case INTAKE_RUNTONEXT:
                if(intakeTimer.seconds()>0.2) {
                    intakeStates = IntakeStates.INTAKE_START;
                }
                break;

            case INTAKE_STOP:
                robot.intakeMotor.setPower(0);
                if(intakeTimer.seconds()>0.1 && intakeTimer.seconds()<0.2 ){
                    robot.spindexerServo.setPosition(0.4);
                }
                if(intakeTimer.seconds()>0.4 && intakeTimer.seconds()<0.5 ){
                    robot.spindexerServo.setPosition(0.3);
                }
                if(intakeTimer.seconds()>0.7 && intakeTimer.seconds()<0.8 ){
                    robot.spindexerServo.setPosition(0.2);
                }
                if(intakeTimer.seconds()>1.0){
                    spindexer.RuntoPosition(0);
                    intakeStates = IntakeStates.INTAKE_IDLE;
                }
                break;

            case INTAKE_REVERSE:
                if (intakeTimer.seconds()>0.5){
                /// stop intake motor for reverse
                robot.intakeMotor.setPower(0);
                intakeStates = IntakeStates.INTAKE_IDLE;
                }
                break;
        }
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
}