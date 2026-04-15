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

    public static IntakeStates intakeStates = IntakeStates.INTAKE_IDLE;

    private ElapsedTime unjamTimer = new ElapsedTime();
    private ElapsedTime jammedTimer = new ElapsedTime();

    public static ElapsedTime intakeTimer = new ElapsedTime();

    private final RobotHardware robot;
    private double intakeRPM;
    private boolean reversing = false;
    SpindexerUpd spindexer;
    private boolean stopMoveRequested = false;

    boolean recorded;

    public FSMIntake(RobotHardware robot, SpindexerUpd spindexer) {
        this.robot = robot;
        this.spindexer = spindexer;
    }

    public void loop() {
        spindexer.updateServoStep();
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
                if (robot.distanceSensor.getDistance(DistanceUnit.MM) < BALL_PRESENT_THRESHOLD_MM) {
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_CAPTURE;
                }
                break;

            case INTAKE_CAPTURE:
                if (intakeTimer.seconds() < 0.05) {
                    spindexer.clearVoteBuffer();
                }
                //nned to make this longer
                else if (intakeTimer.seconds() < 0.2) {
                    // Collect as many samples as possible in 200ms
                    spindexer.addVoteSample(robot.colorSensor, robot.distanceSensor);
                }
                else {
                    // Decide the color of the current slot
                    spindexer.finalizeCurrentSlot();

                    // CHECK: Are all 3 slots filled with something other than Empty?
                    // We check for Green OR Purple. If count is 3, we are full.
                    if (spindexer.count(SpindexerUpd.SLOT.Empty) == 0) {
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
                // OLD - Small delay to allow the servo to physically move before starting the motor again
                if (intakeTimer.seconds() > spindexerServoPerSlotTime) {
                    intakeStates = IntakeStates.INTAKE_START;
                }
                 break;

            case INTAKE_STOP:
                robot.intakeMotor.setPower(ejectSpeed);
                double time = intakeTimer.seconds();
                // Keep your sequence logic for spindexer parking
                if (time > spindexerServoPerSlotTime) {

                    double targetPos =spindexerSlot2;
                    double currentPos = robot.spindexerServo.getPosition();
                    double maxStep = 0.05; // max movement per loop

                    double error = targetPos - currentPos;
                    double step = Math.copySign(
                            Math.min(Math.abs(error), maxStep),
                            error
                    );
                    double nextPos = currentPos + step;
                    robot.spindexerServo.setPosition(nextPos);
                    if (Math.abs(targetPos - nextPos) < 0.01) {
                        spindexer.RuntoPosition(1); // go to slot1 position and reset the spindexer counter
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
        if (intakeStates == IntakeStates.INTAKE_IDLE) return;
        if (intakeStates != IntakeStates.INTAKE_STOP) {
            intakeTimer.reset();
            intakeStates = IntakeStates.INTAKE_STOP;
        }
    }
}