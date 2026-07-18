package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import java.util.Optional;

public class FSMShooter {
    private final RobotHardware robot;
    private final GamepadComboInput gamepadComboInput;
    private ShooterPowerCalculator shooterPowerLUT;
    private final Limelight limelight;



    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime flyWheelTimer = new ElapsedTime();

    public static SHOOTERSTATE shooterState;
    SHOOTERMOTORSTATE shootermotorstate;
    TURRETSTATE turretState;
    SpindexerUpd spindexer;
    public final Turret turret;

    SpindexerUpd.SLOT targetColour = SpindexerUpd.SLOT.Purple;

    private double voltage;
    private double power;                                       //power lut power
    private double angle;                                       //shooter angle

    // shooting sequence config
    private int shootCounter;                                   // counter for # ball shooting
    private long lastFeedTimeMs       = 0;                      // shooting feed time interval time stamp

    LUT<Integer, Long> timeStamp = new LUT<Integer, Long>() {{
        add(1, FEED_PERIOD_MS_CLOSE);
        add(2, FEED_PERIOD_MS_FAR);
    }};

    private long waitTimeMS = 350;

    public double trimTicks;

    // ============================
    // Limelight Tx input (from OpMode loop)
    // ============================
    private boolean llHasTarget = false;
    private double llTxDeg = 0.0;

    // optional smoothing
    private boolean txInit = false;
    private double txFilt = 0.0;

    private double lastValidTx = 0.0;
    private long lastValidTimeMs = 0;

    public static double degToTicks = 2.5;   // tune
    public static int txMaxTicks = 50;
    public static double txDeadbandDeg = 3;

    // Tunables (Dashboard if you like)
    public static double txAlpha = 0.30;// yours
    // since this is trim, keep smaller
    public static long txHoldMs = 120;
    public static long txFadeMs = 200;



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
        SEQUENCE_SHOOTING,
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
                      SpindexerUpd spindexer, ShooterPowerCalculator shooterPowerLUT,
                      GamepadComboInput gamepadComboInput, Turret turret, Limelight limelight) {

        this.robot = robot;
        this.spindexer = spindexer;
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
        trimTicks =0;
    }

    public void SequenceShooterLoop() {
        //===========================================================
        //✅ Add STOPPING to shooter enum
        //✅ Add stopRequested, holdSpindexerPos, shooterTimer
        //✅ Add requestGracefulStop() and canExit()
        //✅ Add enterStoppingState() + handleStoppingState()
        //✅ In shooter loop: if stopRequested → enter STOPPING
        //✅ Transition manager: request stop, wait for canExit()
        //===========================================================

        //==========================================================
        // Set shooter power
        //==========================================================
        voltage = robot.getBatteryVoltageRobust();
        /// shooter motor power controller
        power = shooterPowerLUT.getPower();
        /// shooter motor target rpm, based on distance for fly wheel spool up.
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
        // Update Spindexer servo
        //==========================================================
        spindexer.updateServoStep();

        //==========================================================
        // HANDLE TURRET
        //==========================================================
        boolean aimEnabled =
                shooterState == SHOOTERSTATE.FLYWHEEL_RUNNING ||
                        shooterState == SHOOTERSTATE.KICKER_EXTEND   ||
                        shooterState == SHOOTERSTATE.SEQUENCE_SHOOTING ||
                        shooterState == SHOOTERSTATE.SHOOT_READY;
        turretStateUpdate();

        //get limelight tx adjust
        Limelight.TxSnapshot snap = limelight.getTxForTag(24);
        setLimelightTx(snap.hasTarget, snap.txDeg);
        
        //========================================================
        // NEW Turret Trim
        // Triming/manual control
        //========================================================
        int trimInput=0;
        if (gamepadComboInput.getLbSinglePressedAny()){
            trimInput+=1;
            }
        if (gamepadComboInput.getrbSinglePressedAny()){
            trimInput-=1;
        }
        if (turretState == TURRETSTATE.AIMING && aimEnabled) {
           
            trimTicks =Range.clip(trimTicks +trimInput*trimStep,-400,400);

            int currentTick = turret.getCurrentTick();
            int txAdjustTicks = getTxAdjustTicks();
            int targetTick = (int) (turret.getTargetTick() + trimTicks + txAdjustTicks);

            turret.driveTurretPIDF(currentTick, targetTick);
        }
        else {
            turret.resetTurretProfile();
            robot.turretMotor.setVelocity(trimInput*adjSpeed);
        }
        //=====================================
        // TODO - Update Zone for shooting interval - not in use, - update the MS in main loop right now.
        //=====================================
        int zone  = shooterPowerLUT.getCurrentZone();
        updateZoneForGoalPose(zone);

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
                int targetSlot = 0;
                spindexer.RuntoPosition(targetSlot);
                shooterState = SHOOTERSTATE.KICKER_EXTEND;
                shootTimer.reset();
                flyWheelTimer.reset(); // IMPORTANT: flyWheelTimer should be reset when flywheel starts
                break;

            case KICKER_EXTEND:
                if (shootTimer.seconds() > spindexerServoPerSlotTime) {
                    robot.kickerServo.setPosition(kickerExtend);
                }
                if (shootTimer.seconds() > spindexerServoPerSlotTime*2) {
                    shooterState = SHOOTERSTATE.SHOOT_READY;
                }
                break;
            case SHOOT_READY:
                /// use button Y to shoot.
                if (gamepadComboInput.getYPressedAny()){
                    shooterState = SHOOTERSTATE.SEQUENCE_SHOOTING;
                    shootTimer.reset();
                }
                break;

            case SEQUENCE_SHOOTING:
                boolean flywheelReady =
                        flyWheelTimer.seconds() >= SPOOLUP_SEC ||
                                (robot.topShooterMotor.getVelocity() * SHOOTER_RPM_CONVERSION) >= rpm * 0.95;
                if (!flywheelReady) break;
                // make a timer to feed balls
                long now = System.currentTimeMillis();
                // --- First ball: shoot immediately once flywheel is ready ---
                if (shootCounter == 0) {
                    shootCounter = 1;
                    lastFeedTimeMs = now;
                    spindexer.RunToNext();   // feed 1st ball NOW
                    break;
                }
                // --- Next balls: every FEED_PERIOD_MS ---
                if (now - lastFeedTimeMs >= waitTimeMS) {
                    shootCounter++;
                    lastFeedTimeMs = now;
                    if (shootCounter <= 3) {
                        spindexer.RunToNext();
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
                shootermotorstate = SHOOTERMOTORSTATE.STOP;
                if (shootTimer.seconds() > spindexerServoPerSlotTime) {
                    spindexer.resetSlot();
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.KICKER_RETRACT;
                }
                break;

            case KICKER_RETRACT:
                //=========================================================================
                // this is the place to reset the spindexer counter
                // meanwhile spindexer return back to spinderxerPositions[0] - slot 1 position
                //==========================================================================
                //=========================================================
                // the kicker Retract first then return to slot back to 0 position,
                //=========================================================
                if (shootTimer.seconds() < 0.05){
                    robot.kickerServo.setPosition(kickerRetract);
                }

                if (shootTimer.seconds() > 0.35) {
                    spindexer.RuntoPosition(1); // reset counter in spindexer
                }
                if(shootTimer.seconds()>0.6){
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
        if (gamepadComboInput.getBothTriggersReleasedAny()) {
            turretState = (turretState == TURRETSTATE.LOCKING) ? TURRETSTATE.AIMING : TURRETSTATE.LOCKING;
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

    public double getVoltage () {
        return voltage;
    }

    public double getPower() {
        return power;
    }

    //==========================================================
    //helper - reset trim
    //=========================================================
    public void resetTrim() { trimTicks = 0; }

    //=========================================================
    //limelight tx adjust helper
    //1. set limelightTx by setLimelightTx
    // * Tx snap data class from limelight class, which has two fields: hasTarget boolean and txDeg value
    //2. get txAdjustTicks by getTxAdjustTicks
    // * smooth out the tx value based on the Tx delay time
    // * smooth out the shaking value based on the deadband
    //=========================================================
    public void setLimelightTx(boolean hasTarget, double txDeg) {
        /// limelight Tx snap from limelight class
        /// the Tx snap data class has two fields: hasTarget boolean and txDeg value
        llHasTarget = hasTarget;
        llTxDeg = hasTarget ? txDeg : Double.NaN;
        /// timestamp
        long now = System.currentTimeMillis();
        /// check for valid target
        if (!hasTarget) return;
        /// smooth only when valid
        if (!txInit) { txFilt = txDeg; txInit = true; }
        txFilt = txAlpha * txDeg + (1.0 - txAlpha) * txFilt;
        /// store last valid filtered tx
        lastValidTx = txFilt;
        lastValidTimeMs = now;
    }

    private int getTxAdjustTicks() {
        long now = System.currentTimeMillis();
        double txToUse;
        /// handle missing tx delay, if longer than holdMs, fade to 0 depending on how long it has been
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
        ///  handel shaking tx, compare with deadband to prevent shaking, more than deadband, it drives the turret.
        if (Math.abs(txToUse) < txDeadbandDeg) txToUse = 0.0;

        int adjust = (int) Math.round(-1*txToUse * degToTicks);
        return Range.clip(adjust, -txMaxTicks, txMaxTicks);
    }


}
