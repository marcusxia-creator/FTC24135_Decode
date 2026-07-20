package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;

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
        INTAKE_FINISH,
        INTAKE_REVERSE,
        INTAKE_STOP,
    }
    //States
    public static IntakeStates intakeStates = IntakeStates.INTAKE_IDLE;

    // Subsystems
    private final RobotHardware robot;
    private SpindexerUpd spindexer;

    // time
    private ElapsedTime unjamTimer = new ElapsedTime();
    private ElapsedTime jammedTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();

    // variable holders
    private double intakeRPM;
    private boolean reversing = false;
    private double time;

    // slot variables
    private int targetSlot;
    private int occupiedSlots;

    public FSMIntake(RobotHardware robot, SpindexerUpd spindexer) {
        this.robot = robot;
        this.spindexer = spindexer;
        //this.colorDetection = colorDetection;
    }

    public void loop() {
        spindexer.updateServoStep();
        switch (intakeStates) {
            case INTAKE_IDLE:
                reversing = false;
                robot.intakeMotor.setPower(0);
                break;

            case INTAKE_PREP:
                // Ensure we start at a slot position 1
                targetSlot = 1;
                spindexer.RuntoPosition(targetSlot);
                intakeStates = IntakeStates.INTAKE_START;
                break;

            case INTAKE_START:
                if (robot.spindexerServo.getPosition() - spindexerSlot1 < 0.01) {
                    intakeTimer.reset();
                    occupiedSlots = 0;
                    intakeStates = IntakeStates.INTAKE_CAPTURE;
                }
                break;
            case INTAKE_CAPTURE:
                boolean jammed = isIntakeJammmed();
                HandleIntaking(jammed); // This manages motor power internally
                // Wait for ball to be detected in the intake mouth
                for (SlotSensor sensor : robot.slotSensors) {
                    if (sensor.isBallPresent()) {
                        occupiedSlots++;
                        if (occupiedSlots >= requiredSensors) {
                            robot.intakeMotor.setPower(0);
                            intakeTimer.reset();
                            intakeStates = IntakeStates.INTAKE_FINISH;
                            targetSlot = 0;
                            break;
                        }
                    }
                }
                break;
            case INTAKE_FINISH:
                time = intakeTimer.seconds();
                reversing();
                spindexer.RuntoPosition(0);
                // Keep your sequence logic for spindexer parking
                if (time > spindexerServoPerSlotTime*3) {
                        intakeStates = IntakeStates.INTAKE_IDLE;
                    }
                break;

            case INTAKE_STOP:
                reversing();
                time = intakeTimer.seconds();
                // Keep your sequence logic for spindexer parking
                if (time > spindexerServoPerSlotTime*3) {
                        intakeStates = IntakeStates.INTAKE_IDLE;
                }
                break;

            case INTAKE_REVERSE:
                if (intakeTimer.seconds() < 0.5) {
                    reversing ();
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
        robot.intakeMotor.setPower(ejectSpeed);
    }
    private void HandleIntaking (boolean jammed) {
        if (jammed && !reversing){
            reversing = true;
            unjamTimer.reset();
        }
        if (reversing){
            if (unjamTimer.seconds() > 0.25) {
                reversing();
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