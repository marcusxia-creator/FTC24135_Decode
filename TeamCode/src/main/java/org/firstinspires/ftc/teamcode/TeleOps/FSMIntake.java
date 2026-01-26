package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.BallColor;

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

    private ColorDetection colorDetection;

    private boolean colorDetectionStarted = false;
    private boolean colorRecorded = false;   // prevent writing multiple times
    private BallColor detectedColor = BallColor.UNKNOWN;

    public FSMIntake(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, Spindexer spindexer) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.spindexer = spindexer;

        this.colorDetection = new ColorDetection(robot);
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
                colorDetectionStarted = false;
                colorRecorded = false;
                detectedColor = BallColor.UNKNOWN;
                robot.intakeMotor.setPower(intakeSpeed);
                intakeStates = IntakeStates.INTAKE_START;
                break;
            case INTAKE_START:
                boolean jammed = isIntakeJammmed();

                if (jammed) {
                    HandleIntaking(true);
                } else {
                    robot.intakeMotor.setPower(intakeSpeed);
                }

                if (robot.distanceSensor.getDistance(DistanceUnit.MM) < distanceThreshold) {
                    //recorded = false;
                    intakeTimer.reset();
                    intakeStates = IntakeStates.INTAKE_CAPTURE;
                    ///  --New --
                    colorDetection.startDetection();
                    colorDetectionStarted = true;
                    colorRecorded = false;
                    detectedColor = BallColor.UNKNOWN;
                }
                break;
            //ball goes into spindxer
            case INTAKE_CAPTURE:
                if (intakeTimer.seconds() < 0.05) {
                    spindexer.clearVoteBuffer();
                }
                else if (intakeTimer.seconds() < 0.35) {
                    // Collect as many samples as possible in 250ms
                    spindexer.addVoteSample(robot.colorSensor, robot.distanceSensor);
                }
                else {
                    // Decide the color of the current slot
                    spindexer.finalizeCurrentSlot();

                    // CHECK: Are all 3 slots filled with something other than Empty?
                    // We check for Green OR Purple. If count is 3, we are full.
                    if (spindexer.count(Spindexer.SLOT.Empty) == 0) {
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
                robot.intakeMotor.setPower(0);
                spindexer.RunToNext();
                if (intakeTimer.seconds() > 0.4) {
                    intakeStates = IntakeStates.INTAKE_START;
                    intakeTimer.reset();
                }
                break;

            case INTAKE_STOP:
                if (intakeTimer.seconds()<0.8){
                    robot.intakeMotor.setPower(ejectSpeed);
                }else{
                    robot.intakeMotor.setPower(0);
                }
                double time = intakeTimer.seconds();

                // Keep your sequence logic for spindexer parking
                if (time > 1.0) {
                    spindexer.RuntoPosition(0);
                    intakeStates = IntakeStates.INTAKE_IDLE;
                } else if (time > 0.7) {
                    robot.spindexerServo.setPosition(0.2);
                } else if (time > 0.4) {
                    robot.spindexerServo.setPosition(0.3);
                } else if (time > 0.1) {
                    robot.spindexerServo.setPosition(0.4);
                }
                break;

            case INTAKE_REVERSE:
                if (intakeTimer.seconds()<1.0){
                /// stop intake motor for reverse
                robot.intakeMotor.setPower(ejectSpeed);}
                else{
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