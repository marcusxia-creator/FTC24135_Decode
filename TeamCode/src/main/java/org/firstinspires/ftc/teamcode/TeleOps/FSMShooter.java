package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import java.util.Optional;

public class FSMShooter {
    private final RobotHardware robot;
    private final GamepadComboInput gamepadComboInput;
    private LUTPowerCalculator shooterPowerLUT;
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;

    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime flyWheelTimer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime();

    public static SHOOTERSTATE shooterState;
    SHOOTERMOTORSTATE shootermotorstate;
    TURRETSTATE turretState;
    public final Turret turret;
    private final Limelight limelight;

    private double voltage;
    private double power;   //power lut power
    private double angle;   //shooter angle
    private double power_setpoint=0; //not actually be used
    // shooting sequence config
    private int shootCounter; // counter for # ball shooting
    private long lastFeedTimeMs = 0; // shooting feed time interval time stamp

    LUT<Integer, Long> timeStamp = new LUT<Integer, Long>() {{
        add(1, FEED_PERIOD_MS_CLOSE);
        add(2, FEED_PERIOD_MS_FAR);
    }};

    private long waitTimeMS = 350;

    private boolean LRTriggerBoolean = false;

    public double trim;



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
        SHOOT_READY,
        SHOOTING,
        KICKER_RETRACT,
        SHOOTER_STOP
    }

    public enum SHOOTERMOTORSTATE{
        RUN,
        STOP
    }

    public enum TURRETSTATE {
        AIMING,
        LOCKING
    }

    //Constructor
    public FSMShooter(GamepadEx gamepad_1, GamepadEx gamepad_2, RobotHardware robot,
                      LUTPowerCalculator shooterPowerLUT,
                      GamepadComboInput gamepadComboInput, Turret turret, Limelight limelight) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.shooterPowerLUT = shooterPowerLUT;
        this.gamepadComboInput = gamepadComboInput;
        /// New!!
        this.turret = turret;
        this.limelight = limelight;
    }

    public void Init() {
        robot.topShooterMotor.setPower(0);
        robot.bottomShooterMotor.setPower(0);
        shooterState = SHOOTERSTATE.SHOOTER_IDLE;
        shootermotorstate = SHOOTERMOTORSTATE.STOP;
        turretState = TURRETSTATE.AIMING;
        trim =0;
    }



    public void SequenceShooterLoop() {
        //==========================================================
        // Set shooter power
        //==========================================================
        voltage = robot.getBatteryVoltageRobust();
        /// shooter motor power controller
        power = shooterPowerLUT.getPower(); //get shooter power based on distance Zone and PID+FF power, shooterPowerAngleCalculator.getPower();
        double rpm = shooterPowerLUT.getRPM();

        //==========================================================
        // Set shooter adjuster angle
        //==========================================================
        angle = Range.clip(shooterPowerLUT.getShooterAngle(), 0.12, 0.49); // get shooter adjuster angle.

        //==========================================================
        //Control shooter run/NOT
        //==========================================================
        if (shootermotorstate == SHOOTERMOTORSTATE.RUN){
            robot.topShooterMotor.setPower(power);
            robot.bottomShooterMotor.setPower(power);
            robot.shooterAdjusterServo.setPosition(angle);
        }
        if (shootermotorstate == SHOOTERMOTORSTATE.STOP) {
            robot.topShooterMotor.setPower(0);
            robot.bottomShooterMotor.setPower(0);
        }

        //==========================================================
        // HANDLE TURRET
        //==========================================================
        boolean aimEnabled =
                shooterState == SHOOTERSTATE.FLYWHEEL_RUNNING ||
                        shooterState == SHOOTERSTATE.KICKER_EXTEND   ||
                        shooterState == SHOOTERSTATE.SHOOTING ||
                        shooterState == SHOOTERSTATE.SHOOT_READY;
        turretStateUpdate();

        //========================================================
        // NEW Turret Trim
        // Triming/manual control
        //========================================================
         /** Depreciated!!
        if ((gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                || gamepad_2.getButton(GamepadKeys.Button.LEFT_BUMPER))
                && isButtonDebounced()){
            trimInput+=1;
        }
        if ((gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                || gamepad_2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                && isButtonDebounced()){
            trimInput-=1;
        }

        if (turretState == TURRETSTATE.AIMING && aimEnabled) {
            trim=Range.clip(trim+trimInput*trimStep,-400,400);
            int currentTick = turret.getCurrentTick();
            int targetTick = (int) (turret.getTargetTick() + trim);
            turret.driveTurretPID(currentTick, targetTick);
        }
        else {
            robot.turretMotor.setVelocity(trimInput*adjSpeed);
            ///turret.driveTurretPID(turret.getCurrentTick(), turret.getTargetTick());
        }

        if (turretState == TURRETSTATE.AIMING && aimEnabled) {
            int currentTick = turret.getCurrentTick();
            int targetTick = turret.getTargetTick();
            turret.driveTurretPID(currentTick, targetTick);
        }
        if (turretState == TURRETSTATE.LOCKING) {
            robot.turretMotor.setPower(0);
            ///turret.driveTurretPID(turret.getCurrentTick(), turret.getTargetTick());
        }
         */
         //get limelight tx adjust
        //Limelight.TxSnapshot snap = limelight.getTxForTag(24);
        //setLimelightTx(snap.hasTarget, snap.txDeg);

        int trimInput=0;

        if (turretState == TURRETSTATE.AIMING && aimEnabled) {
            if ((gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                || gamepad_2.getButton(GamepadKeys.Button.LEFT_BUMPER))
                && isButtonDebounced()){
                trimInput+=1;
            }
            if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                || gamepad_2.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && isButtonDebounced()){
                trimInput-=1;
            }
            trim=Range.clip(trim +trimInput*trimStep,-400,400);

            int currentTick = turret.getCurrentTick();
            int targetTick = (int) (turret.getTargetTick() + trim);

            turret.driveTurretPID(currentTick, targetTick);
        }
        else {
            robot.turretMotor.setVelocity(trimInput*adjSpeed);
        }
        //=====================================
        // TODO - Update Zone for shooting interval - not in use, - update the MS in main loop right now.
        //=====================================
        ///int zone  = shooterPowerLUT.getZone();
        ///updateZoneForGoalPose(zone);

        //==========================================================
        // Main FSM
        //==========================================================
        switch (shooterState) {
            case SHOOTER_IDLE:
               //Idle state for shooter
                robot.topShooterMotor.setPower(0);
                robot.bottomShooterMotor.setPower(0);
                shootTimer.reset();
                robot.kickerServo.setPosition(kickerRetract);

                /// New 2.5
                // ✅ single authoritative reset point
                shootCounter = 0;
                lastFeedTimeMs = 0;

                shootTimer.reset();
                flyWheelTimer.reset();
                break;

            case FLYWHEEL_RUNNING:
                // Start flywheels (your motor control elsewhere should respond to this state)
                shootermotorstate = SHOOTERMOTORSTATE.RUN;

                shooterState = SHOOTERSTATE.KICKER_EXTEND;
                shootTimer.reset();
                flyWheelTimer.reset();
                // IMPORTANT: flyWheelTimer should be reset when flywheel starts
                break;

            case KICKER_EXTEND:
                robot.spindexerServo.setPosition(spindexerShootStartPos);
                if (shootTimer.seconds() > 0.3) {
                    robot.kickerServo.setPosition(kickerExtend);
                    shooterState = SHOOTERSTATE.SHOOT_READY;
                }
                break;

            case SHOOT_READY:
                /// use button Y to shoot.
                if ((gamepad_1.getButton(GamepadKeys.Button.Y)
                        || gamepad_2.getButton(GamepadKeys.Button.Y))
                        && isButtonDebounced()){
                    shooterState = SHOOTERSTATE.SHOOTING;
                    shootTimer.reset();
                }
                break;

            case SHOOTING:
                boolean flywheelReady =
                        flyWheelTimer.seconds() >= SPOOLUP_SEC ||
                                (robot.topShooterMotor.getVelocity() * LUTPowerCalculator.tickToRPM) >= rpm * 0.95;
                if (!flywheelReady) break;
                // make a timer to feed balls
                long now = System.currentTimeMillis();
                // --- First ball: shoot immediately once flywheel is ready ---
                if (shootCounter == 0) {
                    shootCounter = 1;
                    lastFeedTimeMs = now;
                    robot.spindexerServo.setPosition(spindexerSlots[1]);   // feed 1st ball NOW
                    break;
                }
                // --- Next balls: every FEED_PERIOD_MS ---
                if (now - lastFeedTimeMs >= waitTimeMS) {
                    shootCounter++;
                    lastFeedTimeMs = now;
                    if (shootCounter <= 3) {
                        robot.spindexerServo.setPosition(spindexerSlots[shootCounter]);
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
                //stop flywheel, very short state, might fuse
                shootermotorstate = SHOOTERMOTORSTATE.STOP;
                shooterState = SHOOTERSTATE.KICKER_RETRACT;
                shootTimer.reset();
                break;

            case KICKER_RETRACT:
                robot.kickerServo.setPosition(kickerRetract);

                if (shootTimer.seconds() > kickerRetractDelay) {
                    robot.spindexerServo.setPosition(spindexerIntakePos); //Park spindexer at intake pos
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                }
                break;

            default:
                shootermotorstate = SHOOTERMOTORSTATE.STOP;
                shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                break;
        }
    }

    //  safe exit in FSMShooter
    public boolean canExit() {
        return shooterState == SHOOTERSTATE.SHOOTER_IDLE;
    }

    //============================================================
    // helper - get turret state
    //============================================================

    public void turretStateUpdate() {
        if (((gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7 && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.7))
        || (gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7 && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.7) && isButtonDebounced() && !LRTriggerBoolean) {
            LRTriggerBoolean = !LRTriggerBoolean;
            if (turretState == TURRETSTATE.LOCKING){
                turretState =  TURRETSTATE.AIMING;
            } else{
                turretState = TURRETSTATE.LOCKING;
            }
        }
    }

    //============================================================
    // helper - get zone
    //============================================================

    public void updateZoneForGoalPose(int zone) {
        int normalizedZone;

        if (zone <= 5) {
            normalizedZone = 1;
        }
        else {
            normalizedZone = 2;
        }

         waitTimeMS = Optional.ofNullable(timeStamp.get(normalizedZone)).orElse(timeStamp.get(1));
    }

    //============================================================
    // helper - get power
    //============================================================

    public double getPower_setpoint () {
        return power_setpoint;
    }

    public double getVoltage () {
        return voltage;
    }

    public double getPower() {
        return power;
    }

    public boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    //=========================================================
    //limelight tx adjust helper
    //1. set limelightTx by setLimelightTx
    // * Tx snap data class from limelight class, which has two fields: hasTarget boolean and tx Deg value
    //2. get txAdjustTicks by getTxAdjustTicks
    // * smooth out the tx value based on the Tx delay time
    // * smooth out the shaking value based on the deadband
    //=========================================================
    /*
    public void setLimelightTx(boolean hasTarget, double txDeg) {
        llHasTarget = hasTarget;
        llTxDeg = hasTarget ? txDeg : Double.NaN;

        long now = System.currentTimeMillis();
        if (!hasTarget) return;

        // smooth only when valid
        if (!txInit) { txFilt = txDeg; txInit = true; }
        txFilt = txAlpha * txDeg + (1.0 - txAlpha) * txFilt;

        // store last valid filtered tx
        lastValidTx = txFilt;
        lastValidTimeMs = now;
    }

    private int getTxAdjustTicks() {
        long now = System.currentTimeMillis();

        double txToUse;

        if (llHasTarget) {
            txToUse = txFilt;
        } else {
            long lostMs = now - lastValidTimeMs;

            if (lostMs <= txHoldMs) {
                txToUse = lastValidTx;                 // hold
            } else if (lostMs <= txHoldMs + txFadeMs) {
                double t = (lostMs - txHoldMs) / (double) txFadeMs; // 0..1
                txToUse = lastValidTx * (1.0 - t);     // fade to 0
            } else {
                txToUse = 0.0;
            }
        }

        if (Math.abs(txToUse) < txDeadbandDeg) txToUse = 0.0;

        int adjust = (int) Math.round(-1*txToUse * degToTicks);
        return Range.clip(adjust, -txMaxTicks, txMaxTicks);
    }
     */
}
