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
    private ColorDetection colorDetection;

    // time
    private ElapsedTime unjamTimer = new ElapsedTime();
    private ElapsedTime jammedTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();

    // variable holders
    private double intakeRPM;
    private boolean reversing = false;
    private double time;

    private boolean stopMoveRequested = false;
    boolean recorded;

    public FSMIntake(RobotHardware robot, SpindexerUpd spindexer, ColorDetection colorDetection) {
        this.robot = robot;
        this.spindexer = spindexer;
        this.colorDetection = colorDetection;
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
                //if (robot.distanceSensor.getDistance(DistanceUnit.MM) < BALL_PRESENT_THRESHOLD_MM) {
                if (colorDetection.isBallPresent()) {
                    intakeTimer.reset();
                    colorDetection.startDetection();
                    intakeStates = IntakeStates.INTAKE_CAPTURE;
                }
                break;

            case INTAKE_CAPTURE:
                /// color detection updates
                colorDetection.updateDetection();
                if (colorDetection.isColorStable()) {
                    /// Write color to current slot
                    spindexer.writeToCurrentSlot(colorDetection.getStableColor());
                    /// Decide to move on to next slot or Finished.
                    if (spindexer.count(SpindexerUpd.SLOT.Empty) == 0) {
                        intakeTimer.reset();
                        intakeStates = IntakeStates.INTAKE_FINISH;
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
                time = intakeTimer.seconds();
                if (time > spindexerServoPerSlotTime) {
                    intakeStates = IntakeStates.INTAKE_START;
                    intakeTimer.reset();
                }
                 break;

            case INTAKE_FINISH:
                robot.intakeMotor.setPower(ejectSpeed);
                time = intakeTimer.seconds();
                // Keep your sequence logic for spindexer parking
                if (time > spindexerServoPerSlotTime) {
                    int targetSlot = 1;
                    /**
                     * double targetPos = spindexerSlot2;
                    double currentPos = spindexer.getServoPosition();
                    //double currentPos = robot.spindexerServo.getPosition();
                    double maxStep = 0.05; // max movement per loop

                    double error = targetPos - currentPos;
                    double step = Math.copySign(
                            Math.min(Math.abs(error), maxStep),
                            error
                    );
                    double nextPos = currentPos + step;
                    //robot.spindexerServo.setPosition(nextPos);
                    spindexer.requestServoPosition(nextPos);

                    if (Math.abs(targetPos - nextPos) < 0.01) {
                        spindexer.RuntoPosition(1); // go to slot1 position and reset the spindexer counter
                        intakeStates = IntakeStates.INTAKE_IDLE;
                    }
                     **/
                    spindexer.requestServoPosition(spindexerPositions[targetSlot]);
                    //spindexer.RuntoPosition(1); // go to slot1 position and reset the spindexer counter
                    if (!spindexer.isServoBusy()){
                        spindexer.setCurrentPos(1);
                        intakeStates = IntakeStates.INTAKE_IDLE;
                    }
                }
                break;

            case INTAKE_STOP:
                robot.intakeMotor.setPower(ejectSpeed);
                time = intakeTimer.seconds();
                // Keep your sequence logic for spindexer parking
                if (time > spindexerServoPerSlotTime) {
                    int targetSlot = 1;
                    spindexer.requestServoPosition(spindexerPositions[targetSlot]);
                    //spindexer.RuntoPosition(1); // go to slot1 position and reset the spindexer counter
                    if (!spindexer.isServoBusy()){
                        spindexer.setCurrentPos(1);
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