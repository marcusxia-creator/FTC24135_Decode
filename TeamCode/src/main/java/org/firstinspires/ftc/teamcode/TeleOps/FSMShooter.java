package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class FSMShooter {
    private ShooterPowerAngleCalculator shooterPowerAngleCalculator;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime rampTimer = new ElapsedTime();
    SHOOTERPOWERSTATE shooterpowerstate = SHOOTERPOWERSTATE.AUTO_POWER;
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
        DETECTING,
        SPINDEXER_ROTATE,
        SHOOTER_STOP
    }

    public Spindexer.Motif motif;

    GamepadManager gamepadManager;

    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, Spindexer spindexer, GamepadManager gamepadManager, ShooterPowerAngleCalculator shooterPowerAngleCalculator) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.spindexer = spindexer;
        this.gamepadManager = gamepadManager;
        this.shooterPowerAngleCalculator = shooterPowerAngleCalculator;
    }

    public void Init() {
        robot.shooterMotor.setPower(0);
        robot.pushRampServo.setPosition(rampDownPos);
        robot.leftGateServo.setPosition(gateDown);
        robot.rightGateServo.setPosition(gateDown);
        shooterState = SHOOTERSTATE.SHOOTER_IDLE;
        robot.shooterMotor.setPower(0);
        motif = spindexer.GPP;//Temporary
    }

    public void ShooterLoop() {
        voltage = robot.getBatteryVoltageRobust();
        speed = shooterPowerAngleCalculator.getPower();
        power_setpoint = (speed*12.0)/voltage;
        // --- Global Controls (can be triggered from any state) ---
        switch (shooterState) {
            case SHOOTER_IDLE:
                //Press B for purple ball
                if (gamepadManager.Purple.PressState) {
                    targetColour = Spindexer.SLOT.Purple;
                }
                //Press A for green ball
                if (gamepadManager.Green.PressState) {
                    targetColour = Spindexer.SLOT.Green;
                }

                // Press 'X' to start spinning the flywheel
                if (gamepadManager.Flywheel.PressState) {
                    //robot.shooterMotor.setVelocity(shooterVel);
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                }

                break;
            case FLYWHEEL_RUNNING:
                ShooterPowerSwitch();
                // Press START an check toggle button true or false to determine slot order for motif
                if (gamepadManager.autoMotif.ToggleState && spindexer.checkMotif(motif)){
                    targetColour=spindexer.motifColour(motif);
                }
                else {
                    //Launch Colour
                    if ((gamepadManager.Purple.PressState) || !spindexer.checkFor(Spindexer.SLOT.Green) && spindexer.checkFor(Spindexer.SLOT.Purple)) {
                        targetColour = Spindexer.SLOT.Purple;
                    }
                    if ((gamepadManager.Green.PressState) || !spindexer.checkFor(Spindexer.SLOT.Purple) && spindexer.checkFor(Spindexer.SLOT.Green)) {
                        targetColour = Spindexer.SLOT.Green;
                    }
                }

                if (spindexer.slotColour()!=targetColour){
                    if(spindexer.checkFor(targetColour)) {
                        shooterState = SHOOTERSTATE.SPINDEXER_ROTATE;
                        shootTimer.reset();
                    }
                    else{
                        shooterState = SHOOTERSTATE.SHOOTER_STOP;
                    }
                }
                // Add shoot condition
                // Press 'Y' to toggle ramp up/down]
                if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()/* && (robot.shooterMotor.getPower() >= (power_setpoint - 0.005) && robot.shooterMotor.getPower() <= (power_setpoint + 0.01)) /*&& shooterVel*shooterFactorThreshold<=robot.shooterMotor.getVelocity()*/) {
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.SHOOTING;
                }
                shootTimer.reset();
                break;
                
            case SHOOTING:
                ShooterPowerSwitch();
                robot.pushRampServo.setPosition(rampUpPos);
                robot.leftGateServo.setPosition(gateUp);
                robot.rightGateServo.setPosition(gateUp);
                if (shootTimer.seconds() > 0.75) {
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.DETECTING;
                }
                break;

            case DETECTING:
                robot.pushRampServo.setPosition(rampDownPos);
                robot.leftGateServo.setPosition(gateDown);
                robot.rightGateServo.setPosition(gateDown);
                if(shootTimer.seconds() > 0.25){
                    double distance = robot.distanceSensor.getDistance(DistanceUnit.MM);
                    if (distance < distanceThreshold) {
                        shootTimer.reset();
                        shooterState = SHOOTERSTATE.SHOOTING;
                    } else {
                        spindexer.writeToCurrent(Spindexer.SLOT.Empty);;
                        shootTimer.reset();
                        if(spindexer.count(Spindexer.SLOT.Empty)==3){
                            shooterState = SHOOTERSTATE.SHOOTER_STOP;
                        }
                        else{
                            shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                        }
                    }
                }
                break;

            case SPINDEXER_ROTATE:
                spindexer.runToSlot(targetColour);
                if (shootTimer.seconds() > 0.45) {
                    shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                }
                break;

            case SHOOTER_STOP:
                robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.shooterMotor.setPower(0);
                robot.leftGateServo.setPosition(gateDown);
                robot.rightGateServo.setPosition(gateDown);
                shooterState=SHOOTERSTATE.SHOOTER_IDLE;

            default:
                robot.shooterMotor.setPower(0);
                shooterState = SHOOTERSTATE.SHOOTER_STOP;
                break;
        }
        // Press 'X' again to stop spinning the flywheel
        if (gamepadManager.Flywheel.PressState && shooterState!=SHOOTERSTATE.SHOOTER_IDLE) {
            robot.shooterMotor.setPower(0);
            shooterState = SHOOTERSTATE.SHOOTER_STOP;
        }

        ShooterPowerControl();
        ShooterPowerSwitch();
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
        if (gamepad_2.getButton(GamepadKeys.Button.LEFT_BUMPER) &&
            gamepad_2.getButton(GamepadKeys.Button.BACK) &&
            isButtonDebounced()) {
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
