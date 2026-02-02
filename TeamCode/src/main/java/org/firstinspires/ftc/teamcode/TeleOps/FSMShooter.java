package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class FSMShooter {
    private final RobotHardware robot;
    private final GamepadInput gamepadInput;
    private LUTPowerCalculator shooterPowerLUT;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;

    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime flyWheelTimer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime();
    private long lastLoopTime = 0;

    SHOOTERSTATE shooterState;
    SORTSHOOTERSTATE sortShooterState;
    SHOOTERMOTORSTATE shootermotorstate;
    SpindexerSimp spindexer;
    Spindexer.SLOT targetColour = Spindexer.SLOT.Purple;

    private double voltage;
    private double power;   //power lut power
    private double angle;   //shooter angle
    private double power_setpoint; //not actually be used
    // shooting sequence config
    private int shootCounter; // counter for # ball shooting
    private long lastFeedTimeMs = 0; // shooting feed time interval time stamp
    private static final long FEED_PERIOD_MS = 600; // 0.6s per feed (tune)
    private static final double SPOOLUP_SEC = 2.25;


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
        /// shooter motor power controller
        power = shooterPowerLUT.getPower(); //get shooter power based on distance Zone and PID+FF power, shooterPowerAngleCalculator.getPower();
        double rpm = shooterPowerLUT.getRPM();
        power_setpoint = (power *13.0)/voltage; // no need to normalized power for shooter to 13v./voltage;
        angle = Range.clip(shooterPowerLUT.getShooterAngle(), 0.06, 0.49); // get shooter adjuster angle.
        /// set shooter adjuster angle
        robot.shooterAdjusterServo.setPosition(angle);
        /// shooter motor state to control shooter run/NOT
        if (shootermotorstate == SHOOTERMOTORSTATE.RUN){
            robot.topShooterMotor.setPower(power);
            robot.topShooterMotor.setPower(power);
        }
        if (shootermotorstate == SHOOTERMOTORSTATE.STOP) {
            robot.topShooterMotor.setPower(0);
            robot.topShooterMotor.setPower(0);
        }

        /// --- FMS ---
        switch (shooterState) {
            case SHOOTER_IDLE:
               //Idle state for shooter
                robot.topShooterMotor.setPower(0);
                robot.bottomShooterMotor.setPower(0);
                shootTimer.reset();
                break;
            case FLYWHEEL_RUNNING:
                shootermotorstate = SHOOTERMOTORSTATE.RUN;
                shootCounter = 0;
                if (shootTimer.seconds() > 0.05){ //wait for flywheel to spool up; needs testing
                    shooterState = SHOOTERSTATE.KICKER_EXTEND;
                    shootTimer.reset();
                    flyWheelTimer.reset();
                }
                break;
            case KICKER_EXTEND:
                    robot.kickerServo.setPosition(kickerExtend);
                    /// use button x again to shoot.
                if ((gamepad_1.getButton(GamepadKeys.Button.X)|| gamepad_2.getButton(GamepadKeys.Button.X) && isButtonDebounced())){
                    robot.spindexerServo.setPosition(spindexerZeroPos);
                    shooterState = SHOOTERSTATE.SEQUENCE_SHOOTING;
                        shootTimer.reset();
                }
                break;
            case SEQUENCE_SHOOTING:
                boolean flywheelReady =
                        flyWheelTimer.seconds() >= SPOOLUP_SEC ||
                                (robot.topShooterMotor.getVelocity() * LUTPowerCalculator.tickToRPM) >= rpm * 0.95;

                if (!flywheelReady) break;

                long now = System.currentTimeMillis();

                // --- First ball: shoot immediately once flywheel is ready ---
                if (shootCounter == 0) {
                    shootCounter = 1;
                    lastFeedTimeMs = now;

                    spindexer.RunToNext();   // feed 1st ball NOW
                    break;
                }

                // --- Next balls: every FEED_PERIOD_MS ---
                if (now - lastFeedTimeMs >= FEED_PERIOD_MS) {
                    shootCounter++;
                    lastFeedTimeMs = now;

                    if (shootCounter < 3) {
                        spindexer.RunToNext();          // feed ball #2, #3
                    } else if (shootCounter == 3) {
                        robot.spindexerServo.setPosition(spindexerSlot4);  // your “end position”
                    } else {
                        shooterState = SHOOTERSTATE.SHOOTER_STOP;
                        shootTimer.reset();

                        // reset these so next time you enter shooting state it works cleanly
                        shootCounter = 0;
                        lastFeedTimeMs = 0;
                    }
                }

                break;

            case SHOOTER_STOP:
                //stop flywheel
               // robot.kickerServo.setPosition(kickerExtend);
                shootermotorstate = SHOOTERMOTORSTATE.STOP;
                if (shootTimer.seconds() > 0.02) {
                    spindexer.resetSlot();
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.KICKER_RETRACT;
                    //shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                }
                break;
            case KICKER_RETRACT:
                if (shootTimer.seconds() > 0.25) {
                    spindexer.RuntoPosition(0);
                }
                if (shootTimer.seconds() > 1.0){
                    robot.kickerServo.setPosition(kickerRetract);
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
        power = shooterPowerLUT.getPower();
        power_setpoint = (power *12.0)/voltage;
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

    public double getPower() {
        return power_setpoint;
    }

    public boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }


}