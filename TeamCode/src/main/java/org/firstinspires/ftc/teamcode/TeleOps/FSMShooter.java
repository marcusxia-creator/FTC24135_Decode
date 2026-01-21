package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.AprilTagMotif.MotifMemorization.motif;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import org.firstinspires.ftc.teamcode.AprilTagMotif.MotifMemorization;

public class FSMShooter {
    private final RobotHardware robot;
    private final GamepadInput gamepadInput;
    private LUTPowerCalculator shooterPowerLUT;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    GamepadManager gamepadManager;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime rampTimer = new ElapsedTime();
    SHOOTERPOWERSTATE shooterpowerstate;
    SHOOTERSTATE shooterState;
    Spindexer spindexer;
    Spindexer.SLOT targetColour = Spindexer.SLOT.Purple;

    private double voltage;
    private double speed;
    private double power_setpoint;

    /**
     * BUTTON FOR SHOOTING
     * * Button X/Square is local key, --- SHOOTER_IDLE STATE---
     *   Press 'X/Square' to start spinning the flywheel
     * * Button X/Square is local key, --- FLYWHEEL STATE---
     *   Cancle the FLYWHEEL within 1 second
     * * Button Y/Triangle is local key, --- RAMPUP STATE---
     *   Press 'Y/Triangle' to toggle ramp up to launch ball to shooter
     */


    public enum SHOOTERSTATE {
        SHOOTER_IDLE,
        FLYWHEEL_RUNNING,
        SHOOTING,
        SPINDEXER_ROTATE,
        SHOOTER_STOP
    }

    //Constructor
    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, Spindexer spindexer, GamepadManager gamepadManager, LUTPowerCalculator shooterPowerLUT,GamepadInput gamepadInput) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.spindexer = spindexer;
        this.gamepadManager = gamepadManager;
        this.shooterPowerLUT = shooterPowerLUT;
        this.gamepadInput = gamepadInput;
    }

    public void Init() {
        robot.shooterMotor.setPower(0);
        robot.pushRampServo.setPosition(rampDownPos);
        robot.leftGateServo.setPosition(gateDown);
        robot.rightGateServo.setPosition(gateDown);
        shooterState = SHOOTERSTATE.SHOOTER_IDLE;
        shooterpowerstate = SHOOTERPOWERSTATE.MANUAL_POWER;
        robot.shooterMotor.setPower(0);

        if(motif==null){
            motif=Spindexer.Motif.GPP;
        }
    }

    public void ShooterLoop() {
        voltage = robot.getBatteryVoltageRobust();
        //speed = shooterPowerAngleCalculator.getPower();
        speed = shooterPowerLUT.getPower();
        power_setpoint = 0.7;//(speed*12.0)/voltage;
        ShooterPowerControl();
        // --- Global Controls (can be triggered from any state) ---
        switch (shooterState) {
            case SHOOTER_IDLE:
                //Press B for purple ball
                if (gamepad_2.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
                    targetColour = Spindexer.SLOT.Purple;
                    shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                    gamepadManager.autoMotif.ToggleState=Boolean.FALSE;
                }
                //Press A for green ball
                if (gamepad_2.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
                    targetColour = Spindexer.SLOT.Green;
                    shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                    gamepadManager.autoMotif.ToggleState=Boolean.FALSE;
                }

                // Press 'X' to start spinning the flywheel
                if (gamepad_2.getButton(GamepadKeys.Button.X) && isButtonDebounced()) {
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                    gamepadManager.autoMotif.ToggleState=Boolean.TRUE;
                }
                break;
            case FLYWHEEL_RUNNING:
                ShooterPowerSwitch();
                // Press START an check toggle button true or false to determine slot order for motif
                // check for targetColor
                if (gamepad_2.getButton(GamepadKeys.Button.B)){
                    gamepadManager.autoMotif.ToggleState=Boolean.TRUE;
                }
                if (gamepadManager.autoMotif.ToggleState && spindexer.checkMotif(motif)){
                    targetColour=spindexer.motifColour(motif);
                }

                if(!spindexer.checkFor(Spindexer.SLOT.Green)||gamepad_2.getButton(GamepadKeys.Button.B)){
                    targetColour = Spindexer.SLOT.Purple;
                    gamepadManager.autoMotif.ToggleState=Boolean.FALSE;
                }
                if(!spindexer.checkFor(Spindexer.SLOT.Purple)||gamepad_2.getButton(GamepadKeys.Button.A)){
                    targetColour = Spindexer.SLOT.Green;
                    gamepadManager.autoMotif.ToggleState=Boolean.FALSE;
                }

                if(spindexer.slotColour()!=targetColour) {
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.SPINDEXER_ROTATE;
                }

                if (gamepad_2.getButton(GamepadKeys.Button.Y)){
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.SHOOTING;
                }
                break;

            case SPINDEXER_ROTATE:
                if (spindexer.slotColour() == targetColour){
                    shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                }
                else{
                    spindexer.runToSlot(targetColour);
                    if (shootTimer.seconds() > 0.5) {
                        shootTimer.reset();
                        shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                    }
                }
                break;

            case SHOOTING:
                ShooterPowerSwitch();
                // Add shoot condition
                // Press 'Y' to toggle ramp up/down]
                robot.pushRampServo.setPosition(rampUpPos);
                robot.leftGateServo.setPosition(gateUp);
                robot.rightGateServo.setPosition(gateUp);
                if (shootTimer.seconds() > 0.5) {
                    robot.pushRampServo.setPosition(rampDownPos);
                    robot.leftGateServo.setPosition(gateDown);
                    robot.rightGateServo.setPosition(gateDown);
                    spindexer.writeToCurrent(Spindexer.SLOT.Empty);
                    if(spindexer.count(Spindexer.SLOT.Empty)==3){
                        shooterState = SHOOTERSTATE.SHOOTER_STOP;
                    }
                    else{
                        shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                    }
                    shootTimer.reset();
                }
                break;

            case SHOOTER_STOP:
                robot.shooterMotor.setPower(0);
                robot.leftGateServo.setPosition(gateDown);
                robot.rightGateServo.setPosition(gateDown);
                shooterState=SHOOTERSTATE.SHOOTER_IDLE;
                break;

            default:
                robot.shooterMotor.setPower(0);
                shooterState = SHOOTERSTATE.SHOOTER_STOP;
                break;
        }

        // Press 'Left Trigger' to stop spinning the flywheel
        if (gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.5 && shooterState!=SHOOTERSTATE.SHOOTER_IDLE) {
            shooterState = SHOOTERSTATE.SHOOTER_STOP;
        }
    }

    public enum SHOOTERPOWERSTATE {
        AUTO_POWER,
        MANUAL_POWER
    }

    public void ToggleShooterPower (){
        if (shooterpowerstate == SHOOTERPOWERSTATE.AUTO_POWER){
            shooterpowerstate = SHOOTERPOWERSTATE.MANUAL_POWER;
        } else {
            shooterpowerstate = SHOOTERPOWERSTATE.AUTO_POWER;
        }
    }

    public void ShooterPowerSwitch () {
        if (shooterpowerstate != SHOOTERPOWERSTATE.AUTO_POWER) {
            robot.shooterMotor.setPower(shooterPower);
        } else {
            robot.shooterMotor.setPower(Range.clip(power_setpoint,0.3,1.0));
        }
    }

    public void ShooterPowerControl () {
        if (/**gamepad_2.getButton(GamepadKeys.Button.LEFT_BUMPER) &&
                gamepad_2.getButton(GamepadKeys.Button.BACK) &&
                isButtonDebounced()*/
        gamepadInput.getOperatorLbBComboPressed()|| gamepadInput.getDriverLbBComboPressed()) {
            ToggleShooterPower();
        }
    }

    public void SetShooterPowerState (SHOOTERPOWERSTATE state) {
        this.shooterpowerstate = state;
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    public double getPower_setpoint () {
        return power_setpoint;
    }

    public double getVoltage () {
        return voltage;
    }

    public double getSpeed () {
        return speed;
    }


}