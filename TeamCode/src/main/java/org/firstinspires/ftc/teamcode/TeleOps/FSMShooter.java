package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.AprilTagMotif.MotifMemorization.motif;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FSMShooter {
    private final RobotHardware robot;
    private final GamepadInput gamepadInput;
    private LUTPowerCalculator shooterPowerLUT;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime rampTimer = new ElapsedTime();
    SHOOTERSTATE shooterState;
    SORTSHOOTERSTATE sortShooterState;
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
        SEQUENCE_SHOOTING,
        SHOOTER_STOP
    }
    public enum SORTSHOOTERSTATE {
        SHOOTER_IDLE,
        FLYWHEEL_RUNNING,
        SORT_SHOOTING,
        SHOOTER_STOP
    }

    //Constructor
    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, Spindexer spindexer, LUTPowerCalculator shooterPowerLUT,GamepadInput gamepadInput) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.spindexer = spindexer;
        this.shooterPowerLUT = shooterPowerLUT;
        this.gamepadInput = gamepadInput;
    }

    public void Init() {
        robot.shooterMotor.setPower(0);
        shooterState = SHOOTERSTATE.SHOOTER_IDLE;
        //shooterpowerstate = SHOOTERPOWERSTATE.AUTO_POWER;
        robot.shooterMotor.setPower(0);

        if(motif==null){
            motif=Spindexer.Motif.GPP;
        }
    }

    public void SequenceShooterLoop() {
        voltage = robot.getBatteryVoltageRobust();
        //speed = shooterPowerAngleCalculator.getPower();
        speed = shooterPowerLUT.getPower();
        power_setpoint = (speed*12.0)/voltage;
        //ShooterPowerControl();
        // --- Global Controls (can be triggered from any state) ---
        switch (shooterState) {
            case SHOOTER_IDLE:
               //Idle state for shooter
                robot.shooterMotor.setPower(0);
                shootTimer.reset();
                break;
            case FLYWHEEL_RUNNING:
                if (shootTimer.seconds() > 0.5){ //wait for flywheel to spool up; needs testing
                    shooterState = SHOOTERSTATE.SEQUENCE_SHOOTING;
                }
                shootTimer.reset();
                break;

            case SEQUENCE_SHOOTING:
                robot.shooterMotor.setPower(speed);
                //Rotate spindexer all the way to shoot
                //Need to replace with spindexer logic
                if (shootTimer.seconds() > 0.1) {
                    robot.leftSpindexerServo.setPosition(spindexerSlot2); //shoot out ball from slot 2
                    robot.rightSpindexerServo.setPosition(spindexerSlot2);
                }
                if (shootTimer.seconds() > 0.3) {
                    robot.leftSpindexerServo.setPosition(spindexerSlot1); //shoot out ball from slot 1
                    robot.rightSpindexerServo.setPosition(spindexerSlot1);
                }
                if (shootTimer.seconds() > 0.5) {
                    robot.leftSpindexerServo.setPosition(spindexerSlot0); //shoot out ball from slot 0
                    robot.rightSpindexerServo.setPosition(spindexerSlot0);
                }
                if (shootTimer.seconds() > 0.7 && robot.distanceSensor.getDistance(DistanceUnit.CM) > 5){
                    shooterState = SHOOTERSTATE.SHOOTER_STOP;
                }
                shootTimer.reset();
                break;

            case SHOOTER_STOP:
                //stop flywheel
                robot.shooterMotor.setPower(0);
                shooterState=SHOOTERSTATE.SHOOTER_IDLE;
                break;

            default:
                robot.shooterMotor.setPower(0);
                shooterState = SHOOTERSTATE.SHOOTER_STOP;
                break;
        }
    }

    public void SortShooterLoop() {
        voltage = robot.getBatteryVoltageRobust();
        //speed = shooterPowerAngleCalculator.getPower();
        speed = shooterPowerLUT.getPower();
        power_setpoint = (speed*12.0)/voltage;
        //ShooterPowerControl();
        switch(sortShooterState) {
            case SHOOTER_IDLE:
                //Idle state for shooter
                robot.shooterMotor.setPower(0);
                shootTimer.reset();
                break;
            case FLYWHEEL_RUNNING:
                if (shootTimer.seconds() > 0.5) {
                    sortShooterState = SORTSHOOTERSTATE.SORT_SHOOTING;
                }
                shootTimer.reset();
                break;
            case SORT_SHOOTING:
                if (shootTimer.seconds() > 0.1) {
                    robot.leftSpindexerServo.setPosition(spindexerSlot2);
                    shootTimer.reset();
                }
                break;
            case SHOOTER_STOP:
                //stop flywheel
                robot.shooterMotor.setPower(0);
                shooterState=SHOOTERSTATE.SHOOTER_IDLE;
                break;
        }
    }
/*
    /// May not need manual power or auto power switch
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
                isButtonDebounced()
        gamepadInput.getOperatorLbBComboPressed()|| gamepadInput.getDriverLbBComboPressed()) {
            ToggleShooterPower();
        }
    }

    public void SetShooterPowerState (SHOOTERPOWERSTATE state) {
        this.shooterpowerstate = state;
    }

 */

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