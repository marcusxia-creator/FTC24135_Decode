package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

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
    SHOOTERMOTORSTATE shootermotorstate;
    SpindexerSimp spindexer;
    Spindexer.SLOT targetColour = Spindexer.SLOT.Purple;

    private double voltage;
    private double speed;
    private double power_setpoint;
    private int shootCounter;

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
        KICKER_EXTEND,
        SEQUENCE_SHOOTING,
        KICKER_RETRACT,
        SHOOTER_STOP
    }
    public enum SORTSHOOTERSTATE {
        SHOOTER_IDLE,
        FLYWHEEL_RUNNING,
        KICKER_EXTEND,
        SORT_SHOOTING,
        SHOOTER_STOP
    }
    public enum SHOOTERMOTORSTATE{
        RUN,
        STOP
    }

    //Constructor
    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot, SpindexerSimp spindexer, LUTPowerCalculator shooterPowerLUT,GamepadInput gamepadInput) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.spindexer = spindexer;
        this.shooterPowerLUT = shooterPowerLUT;
        this.gamepadInput = gamepadInput;
    }

    public void Init() {
        robot.topShooterMotor.setPower(0);
        robot.bottomShooterMotor.setPower(0);
        shooterState = SHOOTERSTATE.SHOOTER_IDLE;
        //shooterpowerstate = SHOOTERPOWERSTATE.AUTO_POWER;
        robot.topShooterMotor.setPower(0);
        robot.bottomShooterMotor.setPower(0);
        shootermotorstate = SHOOTERMOTORSTATE.STOP;

    }

    public void SequenceShooterLoop() {
        voltage = robot.getBatteryVoltageRobust();
        speed = 0.75;
        //speed = shooterPowerLUT.getPower();
        if (shootermotorstate == SHOOTERMOTORSTATE.RUN){
            robot.topShooterMotor.setPower(speed);
            robot.topShooterMotor.setPower(speed);
        }
        if (shootermotorstate == SHOOTERMOTORSTATE.STOP) {
            robot.topShooterMotor.setPower(0);
            robot.topShooterMotor.setPower(0);
        }
        //power_setpoint = (speed*12.0)/voltage;

        //ShooterPowerControl();
        // --- Global Controls (can be triggered from any state) ---
        switch (shooterState) {
            case SHOOTER_IDLE:
               //Idle state for shooter
                robot.topShooterMotor.setPower(0);
                robot.bottomShooterMotor.setPower(0);
                shootTimer.reset();
                //shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                break;
            case FLYWHEEL_RUNNING:
                shootermotorstate = SHOOTERMOTORSTATE.RUN;
                shootCounter = 0;
                if (shootTimer.seconds() > 0.1){ //wait for flywheel to spool up; needs testing
                    shooterState = SHOOTERSTATE.KICKER_EXTEND;
                    shootTimer.reset();
                }
                break;
            case KICKER_EXTEND:
                //Always move to slot 2 after intaking. Add a bit to allow kicker servo to move in{
                    robot.kickerServo.setPosition(kickerExtend);
                if (shootTimer.seconds() > 2.25) {
                    shooterState = SHOOTERSTATE.SEQUENCE_SHOOTING;
                    shootTimer.reset();
                }
                break;
            case SEQUENCE_SHOOTING:
                if (shootTimer.seconds() > 0.5 * shootCounter) {
                    shootCounter++;
                    if (shootCounter < 4) {
                        spindexer.RunToNext();
                    }else{
                        shooterState = SHOOTERSTATE.SHOOTER_STOP;
                        shootTimer.reset();
                    }
                }
                break;
            case SHOOTER_STOP:
                //stop flywheel
               // robot.kickerServo.setPosition(kickerExtend);
                shootermotorstate = SHOOTERMOTORSTATE.STOP;
                if (shootTimer.seconds() > 0.2) {
                    spindexer.resetSlot();
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.KICKER_RETRACT;
                    //shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                }
                break;
            case KICKER_RETRACT:
                if (shootTimer.seconds() > 0.5) {
                    robot.kickerServo.setPosition(kickerRetract);
                }
                if (shootTimer.seconds() > 0.7){
                    spindexer.RuntoPosition(0);
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                }
                break;
            default:
                shootermotorstate = SHOOTERMOTORSTATE.STOP;
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
                robot.topShooterMotor.setPower(0);
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
                    robot.spindexerServo.setPosition(spindexerSlot2);
                    shootTimer.reset();
                }
                break;
            case SHOOTER_STOP:
                //stop flywheel
                robot.topShooterMotor.setPower(0);
                shooterState=SHOOTERSTATE.SHOOTER_IDLE;
                break;
        }
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