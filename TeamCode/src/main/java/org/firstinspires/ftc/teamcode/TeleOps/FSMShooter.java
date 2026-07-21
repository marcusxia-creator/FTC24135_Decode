package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import java.util.Optional;

public class FSMShooter {
    private final RobotHardware robot;
    private final GamepadComboInput gamepadComboInput;
    private LUTPowerCalculator shooterPowerLUT;

    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime flyWheelTimer = new ElapsedTime();

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
    private ElapsedTime loopTimer; // shooting feed time interval time stamp

    LUT<Integer, Long> timeStamp = new LUT<Integer, Long>() {{
        add(1, FEED_PERIOD_MS_CLOSE);
        add(2, FEED_PERIOD_MS_FAR);
    }};

    private long waitTimeMS = 350;

    private boolean txAssistEngaged = false;

    public double trim;

    // =========================================================
    // Limelight TX state
    // =========================================================

    private boolean llHasTarget;

    private boolean txInitialized;
    private double filteredTx;

    private double lastValidTx;
    private long lastValidTimeMs;

    // =========================================================
    // Dashboard Limelight tuning
    // =========================================================

    public static double txdegToTicks = 2.4; //
    public static int txMaxTicks = 50;
    public static double txDeadbandDeg = 3.0;

    public static double txAlpha = 0.30;

    public static long txHoldMs = 120;
    public static long txFadeMs = 200;

    public static int txAssistEngageTicks = 13;
    public static int txAssistDisengageTicks = 15;


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
    public FSMShooter(RobotHardware robot,
                      LUTPowerCalculator shooterPowerLUT,
                      GamepadComboInput gamepadComboInput, Turret turret, Limelight limelight) {
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
        loopTimer = new ElapsedTime();
        llHasTarget = false;
        txInitialized = false;
        filteredTx = 0.0;
        lastValidTx = 0.0;
        lastValidTimeMs = 0L;
    }



    public void SequenceShooterLoop() {
        //==========================================================
        // Set shooter power
        //==========================================================

        /// shooter motor power controller
        power = shooterPowerLUT.getPower(); //get shooter power based on distance Zone and PID+FF power, shooterPowerAngleCalculator.getPower();
        double rpm = shooterPowerLUT.getRPM();

        //==========================================================
        // Set shooter adjuster angle
        //==========================================================
        angle = Range.clip(shooterPowerLUT.getShooterAngle(), 0.12, 0.49); // get shooter adjuster angle.
        robot.shooterAdjusterServo.setPosition(angle);
        //==========================================================
        //Control shooter run/NOT
        //==========================================================
        boolean shooterEnabled = shootermotorstate == SHOOTERMOTORSTATE.RUN;
        if (shooterEnabled){
            robot.topShooterMotor.setPower(power);
            robot.bottomShooterMotor.setPower(power);
        } else {
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

        int trimInput=0;

        if (turretState == TURRETSTATE.AIMING && aimEnabled) {
            //set limelight
            Limelight.TxSnapshot snap = limelight.getTxForTag(24);
            setLimelightTx(snap.hasTarget, snap.txDeg);

            if (gamepadComboInput
                    .getLbSinglePressedAny()){
                trimInput+=1;
            }
            if (gamepadComboInput
                    .getrbSinglePressedAny()){
                trimInput-=1;
            }
            trim=Range.clip(trim +trimInput*trimStep,-400,400);

            // tx from limelight
            int txAdjustTicks =
                    getTxAdjustTicks();

            int currentTick = turret.getCurrentTick();
            int baseTargetTick = (int) (turret.getTargetTick() + trim);
            int tickError = Math.abs(baseTargetTick - currentTick);

            // Hysteresis: only start tx-assist once close in (engage), and only
            // drop it if it drifts well past that point (disengage) — prevents
            // chatter right at the boundary between coarse and fine aiming.
            if (txAssistEngaged) {
                if (tickError > txAssistDisengageTicks) {
                    txAssistEngaged = false;
                }
            } else {
                if (tickError <= txAssistEngageTicks) {
                    txAssistEngaged = true;
                }
            }

            int targetTick = txAssistEngaged ? baseTargetTick + txAdjustTicks : baseTargetTick;

            turret.driveTurretPID(currentTick, targetTick, loopTimer.seconds());
            loopTimer.reset();
        }
        else {
            setLimelightTx(false, Double.NaN);
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
                shootermotorstate = SHOOTERMOTORSTATE.STOP;
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
                /// 07/18/2026 Set spindexer to shooter start slot pos
                robot.spindexerServo.setPosition(spindexerShootStartPos);
                if (shootTimer.seconds()>0.3) {
                    shooterState = SHOOTERSTATE.KICKER_EXTEND;
                    shootTimer.reset();
                    flyWheelTimer.reset();
                }
                // IMPORTANT: flyWheelTimer should be reset when flywheel starts
                break;

            case KICKER_EXTEND:
                /** 07/18/2026 moved to FLYWHEEL_RUNNING stat
                *robot.spindexerServo.setPosition(spindexerShootStartPos);
                 * */
                    robot.kickerServo.setPosition(kickerExtend);
                    shooterState = SHOOTERSTATE.SHOOT_READY;
                break;

            case SHOOT_READY:
                /// use button Y to shoot.
                if (gamepadComboInput.getYPressedAny()){
                    shooterState = SHOOTERSTATE.SHOOTING;
                    shootTimer.reset();
                }
                break;

            case SHOOTING:
                double measuredRPM = shooterPowerLUT.getMeasureRPM();
                boolean flywheelReady =
                        flyWheelTimer.seconds() >= SPOOLUP_SEC ||
                                measuredRPM  >= rpm * 0.95;
                if (!flywheelReady) return;
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
                        lastFeedTimeMs = 0L;
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
        if (gamepadComboInput.getBothTriggersRisingEdgeAny()) {
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

    //=========================================================
    //limelight tx adjust helper
    //1. set limelightTx by setLimelightTx
    // * Tx snap data class from limelight class, which has two fields: hasTarget boolean and tx Deg value
    //2. get txAdjustTicks by getTxAdjustTicks
    // * smooth out the tx value based on the Tx delay time
    // * smooth out the shaking value based on the deadband
    //=========================================================

    public void setLimelightTx(
            boolean hasTarget,
            double txDegrees
    ) {
        llHasTarget = hasTarget && Double.isFinite(txDegrees);

        long now = System.currentTimeMillis();

        if (!llHasTarget) {
            return;
        }

        double safeAlpha = Range.clip(txAlpha,0.0,1.0);

        if (!txInitialized) {
            filteredTx = txDegrees;
            txInitialized = true;
        } else {
            filteredTx = safeAlpha * txDegrees + (1.0 - safeAlpha) * filteredTx;
        }

        lastValidTx = filteredTx;

        lastValidTimeMs = now;
    }

    private int getTxAdjustTicks() {
        long now = System.currentTimeMillis();

        double txToUse;

        if (llHasTarget) {
            txToUse = filteredTx;
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

        int adjust = (int) Math.round(-1*txToUse * txdegToTicks);
        return Range.clip(adjust, -txMaxTicks, txMaxTicks);
    }
    // =========================================================
    // Get LimelightTxFOR LED

    public double getLimelightTxForLED() {
        if (!llHasTarget) {
            return Double.NaN;
        }

        return filteredTx;
    }

    public boolean hasLimelightTarget() {
        return llHasTarget;
    }
}
