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
    SpindexerSimp spindexer;

    boolean recorded;

    public FSMIntake(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, SpindexerSimp spindexer) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;

        this.spindexer = spindexer;
    }

    public void loop() {
        switch (intakeStates) {
            case INTAKE_IDLE:
                reversing = false;
                robot.intakeMotor.setPower(0);
                break;

            case INTAKE_PREP:
                // Ensure we start at a clean position
                spindexer.RuntoPosition(0);
                intakeTimer.reset();
                intakeStates = IntakeStates.INTAKE_START;
                break;

            case INTAKE_START:
                boolean jammed = isIntakeJammmed();
                HandleIntaking(jammed); // This manages motor power internally

                // Wait for ball to be detected in the intake mouth
                if (robot.distanceSensor.getDistance(DistanceUnit.MM) < distanceThreshold) {
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_CAPTURE;
                }
                break;

            case INTAKE_CAPTURE:
                if (intakeTimer.seconds() < 0.05) {
                    spindexer.clearVoteBuffer();
                }
                //nned to make this longer
                else if (intakeTimer.seconds() < 0.4) {
                    // Collect as many samples as possible in 250ms
                    spindexer.addVoteSample(robot.colorSensor, robot.distanceSensor);
                }
                else {
                    // Decide the color of the current slot
                    spindexer.finalizeCurrentSlot();

                    // CHECK: Are all 3 slots filled with something other than Empty?
                    // We check for Green OR Purple. If count is 3, we are full.
                    if (spindexer.count(SpindexerSimp.SLOT.Empty) == 0) {
                        intakeTimer.reset();
                        intakeStates = IntakeStates.INTAKE_STOP;
                    } else {
                        // Move to next physical slot and wait for next ball
                        spindexer.RunToNext();
                        intakeTimer.reset();
                        intakeStates = IntakeStates.INTAKE_RUNTONEXT;
                    }
                }
                break;

            case INTAKE_RUNTONEXT:
                // Small delay to allow the servo to physically move before starting the motor again
                if (intakeTimer.seconds() > 0.2) {
                    intakeStates = IntakeStates.INTAKE_START;
                }
                break;

            case INTAKE_STOP:
                robot.intakeMotor.setPower(ejectSpeed);
                double time = intakeTimer.seconds();

                // Keep your sequence logic for spindexer parking

                if (time > 0.15) {
                        /**
                        spindexer.RuntoPosition(0);
                        intakeStates = IntakeStates.INTAKE_IDLE;
                    } else if (time > 0.6) {
                        robot.spindexerServo.setPosition(0.19);
                    } else if (time > 0.4) {
                        robot.spindexerServo.setPosition(0.29);
                    } else if (time > 0.2) {
                        robot.spindexerServo.setPosition(0.39);
                    }*/
                    double targetPos = spindexerSlot1;
                    double currentPos = robot.spindexerServo.getPosition();
                    double maxStep = 0.05; // max movement per loop

                    double error = targetPos - currentPos;
                    double step = Math.copySign(
                            Math.min(Math.abs(error), maxStep),
                            error
                    );

                    robot.spindexerServo.setPosition(currentPos + step);

                    if (Math.abs(error) < 0.005) {
                        spindexer.RuntoPosition(0);
                        intakeStates = IntakeStates.INTAKE_IDLE;
                        }
                }
                break;

            case INTAKE_REVERSE:
                if (intakeTimer.seconds() < 0.5) {
                    robot.intakeMotor.setPower(ejectSpeed);
                }else{
                    intakeStates = IntakeStates.INTAKE_IDLE;
                }
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
}