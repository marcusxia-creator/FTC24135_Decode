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


    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime flyWheelTimer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime();
    private long lastLoopTime = 0;

    public static SHOOTERSTATE shooterState;
    SORTSHOOTERSTATE sortShooterState;
    SHOOTERMOTORSTATE shootermotorstate;
    TURRETSTATE turretState;
    SpindexerUpd spindexer;
    public final Turret turret;

    Spindexer.SLOT targetColour = Spindexer.SLOT.Purple;

    private double voltage;
    private double power;   //power lut power
    private double angle;   //shooter angle
    private double power_setpoint=0; //not actually be used
    // shooting sequence config
    private int shootCounter; // counter for # ball shooting
    private long lastFeedTimeMs = 0; // shooting feed time interval time stamp

    private final ElapsedTime shooterTimer = new ElapsedTime();
    public boolean stopRequested = false;

    // Holds the current spindexer servo position when stop is requested
    private double holdSpindexerPos = 0.0;
    private boolean stopInitDone = false;

    // Tuning constants
    private double stopClearancePos = 0;
    private boolean clearanceChosen = false;

    private static final double MOVE_TO_CLEARANCE_TIME_S = 0.2;  // tune
    private static final double KICKER_RETRACT_TIME_S     = 0.25;  // tune
    private static final double PARK_TO_ZERO_TIME_S       = 0.50;  // tune


    LUT<Integer, Long> timeStamp = new LUT<Integer, Long>() {{
        add(1, FEED_PERIOD_MS_CLOSE);
        add(2, FEED_PERIOD_MS_FAR);
    }};

    private long waitTimeMS = 350;

    private boolean LRTriggerBoolean = false;

    public double trim = 0;

    // ============================
    // Limelight Tx input (from OpMode loop)
    // ============================
    private boolean llHasTarget = false;
    private double llTxDeg = 0.0;

    // optional smoothing
    private boolean txInit = false;
    private double txFilt = 0.0;
    public static double txAlpha = 0.25;   // tune 0.15~0.35

    public static double degToTicks = 2.5;   // tune
    public static int txMaxTicks = 400;
    public static double txDeadbandDeg = 0.4;


    /**
     * BUTTON FOR SHOOTING
     * * Button X/Square is global key, --- SHOOTER_IDLE STATE---
     *   Press 'X/Square' to start spinning the flywheel
     * * Button Y/Triangle is local key, --- SHOOT STATE---
     *   Press 'Y/Triangle' to toggle spindexer run to launch ball to shooter
     */


    public enum SHOOTERSTATE {
        SHOOTER_IDLE,
        FLYWHEEL_RUNNING,
        KICKER_EXTEND,
        SHOOT_READY,
        SEQUENCE_SHOOTING,
        KICKER_RETRACT,
        SHOOTER_STOP,
        SHOOTER_GRACE_STOPPING
    }
    public enum SORTSHOOTERSTATE {
        SHOOTER_IDLE,
        FLYWHEEL_RUNNING,
        KICKER_EXTEND,
        SORT_SHOOTING,
        SHOOTER_STOP,
        SHOOTER_GRACE_STOPPING
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
                      SpindexerUpd spindexer, LUTPowerCalculator shooterPowerLUT,
                      GamepadComboInput gamepadComboInput, Turret turret) {

        this.robot = robot;
        this.spindexer = spindexer;
        this.shooterPowerLUT = shooterPowerLUT;
        this.gamepadComboInput = gamepadComboInput;
        /// New!!
        this.turret = turret;
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
        // Handle stopping
        // If stop is requested, immediately transition to STOPPING
        //==========================================================
        if (stopRequested
                && shooterState != SHOOTERSTATE.SHOOTER_IDLE
                && shooterState != SHOOTERSTATE.SHOOTER_GRACE_STOPPING) {
                enterStoppingState();   // this sets stopInitDone = true
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

        //========================================================
        // NEW Turret Trim
        // Triming/manual control
        //========================================================
        int trimInput=0;
        int homeTick = turret.getTurretHomeTick();

        if (turretState == TURRETSTATE.AIMING && aimEnabled) {
            if (gamepadComboInput.getLbSinglePressedAny()){
                trimInput+=1;
            }
            if (gamepadComboInput.getrbSinglePressedAny()){
                trimInput-=1;
            }
            trim=Range.clip(trim+trimInput*trimStep,-400,400);
            int currentTick = turret.getCurrentTick();
            int txAdjust = getTxAdjustTicks();
            int targetTick = (int) (turret.getTargetTick() + trim + homeTick+txAdjust);
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

                stopRequested = false;
                stopInitDone = false;

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
                robot.kickerServo.setPosition(kickerExtend);
                if (shootTimer.seconds() > 0.15) {
                    double currentPosition = robot.spindexerServo.getPosition();
                    spindexer.requestServoPosition(currentPosition - 0.05);
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
                                (robot.topShooterMotor.getVelocity() * LUTPowerCalculator.tickToRPM) >= rpm * 0.95;
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
                    if (shootCounter < 3) {
                        spindexer.RunToNext();          // feed ball #2, #3
                    } else if (shootCounter == 3) {
                        spindexer.requestServoPosition(spindexerSlot5); // your “end position”
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
                // when slot back to 0 position,then the kicker Retract
                //=========================================================
                if (shootTimer.seconds() > 0.4){
                    robot.kickerServo.setPosition(kickerRetract);
                }

                if (shootTimer.seconds() > 0.8) {
                    spindexer.RuntoPosition(0); // reset counter in spindexer
                    shootTimer.reset();
                    shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                }
                break;

            case SHOOTER_GRACE_STOPPING:
                handleStoppingState();
                break;

            default:
                shootermotorstate = SHOOTERMOTORSTATE.STOP;
                shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                break;
        }
    }

    //=========================================================
    // HANDLE STOPPING
    //=========================================================
    private void enterStoppingState() {
        shooterState = SHOOTERSTATE.SHOOTER_GRACE_STOPPING;
        if (!stopInitDone){
            shooterTimer.reset();
            // Freeze spindexer exactly where it currently is
            holdSpindexerPos = robot.spindexerServo.getPosition();

            // Mark init done so we don't recapture every loop
            stopInitDone = true;
        }
    }

    private void handleStoppingState() {
        // Always stop flywheel immediately
        shootermotorstate = SHOOTERMOTORSTATE.STOP;
        robot.topShooterMotor.setPower(0.0);
        robot.bottomShooterMotor.setPower(0.0);

        double t = shooterTimer.seconds();
        // ---------------------------------------------------------
        // Step 0: Decide clearance slot ONCE (closest > current)
        // ---------------------------------------------------------
        if (!clearanceChosen) {
            int emptyCount = spindexer.count(SpindexerUpd.SLOT.Empty);
            boolean noBall = emptyCount == 3; // =   noBall

            if (noBall) {
                stopClearancePos = spindexerPositions[0];
                clearanceChosen = true;
            } else {
                double next = spindexer.findNextHigherSlot(holdSpindexerPos, spindexerPositions);
                if (next < 0) {
                    // If no higher slot exists, choose a safe fallback:
                    // Option A: go to the last slot
                    stopClearancePos = spindexerPositions[spindexerPositions.length - 1];
                    // Option B (alternative): stopClearancePos = spindexerPositions[0]; // wrap
                } else {
                    stopClearancePos = next;
                }
                clearanceChosen = true;
            }
        }

        // ---------------------------------------------------------
        // Step 1: Move spindexer to clearance slot (do NOT retract yet)
        // ---------------------------------------------------------
        if (t < MOVE_TO_CLEARANCE_TIME_S) {
            int p = Range.clip( (int) (stopClearancePos/slotAngleDelta),0,3);
            spindexer.RuntoPosition(p);
            return;
        }

        // ---------------------------------------------------------
        // Step 2: Retract kicker (now clearance achieved)
        // ---------------------------------------------------------
        if (t < (MOVE_TO_CLEARANCE_TIME_S + KICKER_RETRACT_TIME_S)) {
            robot.kickerServo.setPosition(kickerRetract);
            robot.spindexerServo.setPosition(stopClearancePos); // keep holding clearance
            return;
        }

        // ---------------------------------------------------------
        // Step 3: Park spindexer to 0 (home)
        // ---------------------------------------------------------
        if (t < (MOVE_TO_CLEARANCE_TIME_S + KICKER_RETRACT_TIME_S + PARK_TO_ZERO_TIME_S)) {
            robot.kickerServo.setPosition(kickerRetract);
            spindexer.RuntoPosition(0);  // reset spindexer slot counter to 0
            return;
        }
        // ---------------------------------------------------------
        // Done
        // ---------------------------------------------------------
        stopRequested = false;
        stopInitDone = false;
        clearanceChosen = false;
        shooterState = SHOOTERSTATE.SHOOTER_IDLE;
    }

    //  safe exit in FSMShooter
    public boolean canExit() {
        return shooterState == SHOOTERSTATE.SHOOTER_IDLE;
    }

    public void requestGracefulStop() {
        // If you have a specific STOPPING state, use it here.
        stopRequested= true;
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

    public double getPower_setpoint () {
        return power_setpoint;
    }

    public double getVoltage () {
        return voltage;
    }

    public double getPower() {
        return power;
    }

    //==========================================================
    //helper - reset trim
    //=========================================================
    public void resetTrim (){
        trim = 0;
    }
    //=========================================================
    //limelight tx adjust
    //=========================================================
    public void setLimelightTx(boolean hasTarget, double txDeg) {
        llHasTarget = hasTarget;
        llTxDeg = txDeg;

        // smooth only when we have a target
        if (hasTarget) {
            if (!txInit) { txFilt = txDeg; txInit = true; }
            txFilt = txAlpha * txDeg + (1.0 - txAlpha) * txFilt;
        }
    }

    private int getTxAdjustTicks() {
        if (!llHasTarget) return 0;

        double tx = txFilt; // or llTxDeg if you don't want smoothing
        if (Math.abs(tx) < txDeadbandDeg) tx = 0.0;

        int adjust = (int) Math.round(tx * degToTicks);
        return Range.clip(adjust, -txMaxTicks, txMaxTicks);
    }
}