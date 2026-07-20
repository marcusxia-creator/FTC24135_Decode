package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class FSMShooter {

    private final RobotHardware robot;
    private final GamepadComboInput gamepadComboInput;
    private final ShooterPowerCalculator shooterPowerCalculator;
    private final Limelight limelight;
    private final SpindexerUpd spindexer;

    public final TurretUpd turret;

    private final ElapsedTime shootTimer =
            new ElapsedTime();

    private final ElapsedTime flywheelTimer =
            new ElapsedTime();

    public static SHOOTERSTATE shooterState;

    private SHOOTERMOTORSTATE shooterMotorState;
    private TURRETSTATE turretState;

    private double voltage;
    private double power;
    private double angle;

    private int shootCounter;
    private long lastFeedTimeMs;
    private long waitTimeMs;

    public double trimTicks;

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

    public static double degToTicks = 2.5;
    public static int txMaxTicks = 50;
    public static double txDeadbandDeg = 3.0;

    public static double txAlpha = 0.30;

    public static long txHoldMs = 120;
    public static long txFadeMs = 200;

    // =========================================================
    // FSM states
    // =========================================================

    public enum SHOOTERSTATE {
        SHOOTER_IDLE,
        FLYWHEEL_RUNNING,
        KICKER_EXTEND,
        SHOOT_READY,
        SEQUENCE_SHOOTING,
        KICKER_RETRACT,
        SHOOTER_STOP
    }

    public enum SHOOTERMOTORSTATE {
        RUN,
        STOP
    }

    public enum TURRETSTATE {
        AIMING,
        LOCKING
    }

    // =========================================================
    // Constructor
    // =========================================================

    public FSMShooter(
            RobotHardware robot,
            SpindexerUpd spindexer,
            ShooterPowerCalculator shooterPowerCalculator,
            GamepadComboInput gamepadComboInput,
            TurretUpd turret,
            Limelight limelight
    ) {
        this.robot = robot;
        this.spindexer = spindexer;
        this.shooterPowerCalculator =
                shooterPowerCalculator;
        this.gamepadComboInput =
                gamepadComboInput;
        this.turret = turret;
        this.limelight = limelight;
    }

    // =========================================================
    // Initialization
    // =========================================================

    public void Init() {
        robot.topShooterMotor.setPower(0.0);
        robot.bottomShooterMotor.setPower(0.0);

        shooterState =
                SHOOTERSTATE.SHOOTER_IDLE;

        shooterMotorState =
                SHOOTERMOTORSTATE.STOP;

        turretState =
                TURRETSTATE.AIMING;

        trimTicks = 0.0;

        shootCounter = 0;
        lastFeedTimeMs = 0L;

        llHasTarget = false;
        txInitialized = false;
        filteredTx = 0.0;
        lastValidTx = 0.0;
        lastValidTimeMs = 0L;

        shootTimer.reset();
        flywheelTimer.reset();

        turret.resetTurretProfile();
    }

    // =========================================================
    // Main FSM loop
    // =========================================================

    /**
     * Call once during every TeleOp loop.
     *
     * Before this method, the main TeleOp should:
     *
     * 1. Clear the REV Hub bulk cache
     * 2. Call robot.pinpoint.update()
     */
    public void SequenceShooterLoop() {

        /*
         * Handle turret aiming/locking button.
         */
        turretStateUpdate();

        /*
         * Process the shooter FSM first.
         *
         * This updates shooterMotorState before motor power is
         * written, avoiding a one-loop start/stop delay.
         */
        updateShooterFSM();

        /*
         * ShooterPowerCalculator:
         *
         * - Updates distance and zone
         * - Updates target RPM and adjuster angle
         * - Updates Dashboard PID gains
         * - Calculates PID + feedforward power
         * - Resets PID when shooterMotorState is STOP
         */
        boolean shooterEnabled =
                shooterMotorState
                        == SHOOTERMOTORSTATE.RUN;

        power =
                shooterPowerCalculator.getPower(
                        shooterEnabled
                );

        voltage =
                robot.getBatteryVoltageRobust();

        angle =
                Range.clip(
                        shooterPowerCalculator
                                .getShooterAngle(),
                        0.12,
                        0.49
                );

        /*
         * Set the shooter adjuster every loop so it is in
         * position before firing.
         */
        robot.shooterAdjusterServo.setPosition(angle);

        /*
         * Apply shooter motor output.
         */
        if (shooterEnabled) {
            robot.topShooterMotor.setPower(power);
            robot.bottomShooterMotor.setPower(power);
        } else {
            robot.topShooterMotor.setPower(0.0);
            robot.bottomShooterMotor.setPower(0.0);
        }

        /*
         * Update turret target and cached sensor snapshot after
         * ShooterPowerCalculator updates the current zone.
         */
        updateTurret();

        /*
         * Apply any new Spindexer target generated by the FSM.
         */
        spindexer.updateServoStep();
    }

    // =========================================================
    // Shooter FSM
    // =========================================================

    private void updateShooterFSM() {
        switch (shooterState) {

            case SHOOTER_IDLE:
                shooterMotorState =
                        SHOOTERMOTORSTATE.STOP;

                robot.kickerServo.setPosition(kickerRetract);

                /*
                 * Single authoritative reset location.
                 */
                shootCounter = 0;
                lastFeedTimeMs = 0L;

                shootTimer.reset();
                flywheelTimer.reset();
                break;

            case FLYWHEEL_RUNNING:
                shooterMotorState = SHOOTERMOTORSTATE.RUN;

                /*
                 * Move Spindexer to its starting slot.
                 */
                spindexer.RuntoPosition(0);

                shooterState = SHOOTERSTATE.KICKER_EXTEND;

                shootTimer.reset();
                flywheelTimer.reset();
                break;

            case KICKER_EXTEND:
                shooterMotorState = SHOOTERMOTORSTATE.RUN;

                if (shootTimer.seconds()
                        > spindexerServoPerSlotTime) {

                    robot.kickerServo.setPosition(
                            kickerExtend
                    );
                }

                if (shootTimer.seconds()
                        > spindexerServoPerSlotTime * 2.0) {

                    shooterState =
                            SHOOTERSTATE.SHOOT_READY;
                }
                break;

            case SHOOT_READY:
                shooterMotorState =
                        SHOOTERMOTORSTATE.RUN;

                if (gamepadComboInput.getYPressedAny()) {
                    shooterState =
                            SHOOTERSTATE.SEQUENCE_SHOOTING;

                    shootTimer.reset();
                }
                break;

            case SEQUENCE_SHOOTING:
                shooterMotorState =
                        SHOOTERMOTORSTATE.RUN;

                updateSequenceShooting();
                break;

            case SHOOTER_STOP:
                /*
                 * This state change is applied to the motors in
                 * the current loop, not the next loop.
                 */
                shooterMotorState =
                        SHOOTERMOTORSTATE.STOP;

                if (shootTimer.seconds()
                        > spindexerServoPerSlotTime) {

                    spindexer.resetSlot();

                    shootTimer.reset();

                    shooterState =
                            SHOOTERSTATE.KICKER_RETRACT;
                }
                break;

            case KICKER_RETRACT:
                shooterMotorState =
                        SHOOTERMOTORSTATE.STOP;

                double retractTime =
                        shootTimer.seconds();

                if (retractTime < 0.05) {
                    robot.kickerServo.setPosition(
                            kickerRetract
                    );
                }

                if (retractTime > 0.35) {
                    spindexer.RuntoPosition(1);
                }

                if (retractTime > 0.6) {
                    shootTimer.reset();

                    shooterState =
                            SHOOTERSTATE.SHOOTER_IDLE;
                }
                break;

            default:
                shooterMotorState =
                        SHOOTERMOTORSTATE.STOP;

                shooterState =
                        SHOOTERSTATE.SHOOTER_IDLE;
                break;
        }
    }

    // =========================================================
    // Ball shooting sequence
    // =========================================================

    private void updateSequenceShooting() {
        double targetRPM =
                shooterPowerCalculator.getRPM();

        double measuredRPM =
                shooterPowerCalculator.getMeasureRPM();

        /*
         * The target and measured RPM values are from the
         * previous calculator update, which occurred during the
         * previous loop. That is sufficient for the ready check.
         */
        boolean flywheelAtSpeed =
                targetRPM > 0.0
                        && measuredRPM
                        >= targetRPM * 0.95;

        boolean spoolupTimedOut =
                flywheelTimer.seconds()
                        >= SPOOLUP_SEC;

        boolean flywheelReady =
                flywheelAtSpeed
                        || spoolupTimedOut;

        if (!flywheelReady) {
            return;
        }

        long now =
                System.currentTimeMillis();

        /*
         * First ball is fed immediately after the flywheel is
         * ready.
         */
        if (shootCounter == 0) {
            shootCounter = 1;
            lastFeedTimeMs = now;

            spindexer.RunToNext();

            return;
        }

        if (now - lastFeedTimeMs
                < waitTimeMs) {
            return;
        }

        shootCounter++;
        lastFeedTimeMs = now;

        if (shootCounter <= 3) {
            spindexer.RunToNext();
        } else {
            shooterState =
                    SHOOTERSTATE.SHOOTER_STOP;

            /*
             * Stop state timing starts now.
             */
            shootTimer.reset();

            shootCounter = 0;
            lastFeedTimeMs = 0L;
        }
    }

    // =========================================================
    // Turret handling
    // =========================================================

    private void updateTurret() {
        int zone =
                shooterPowerCalculator
                        .getCurrentZone();

        /*
         * Update feed interval directly from Dashboard config.
         *
         * This avoids a LUT that captures old values when the
         * class is constructed.
         */
        waitTimeMs =
                getTimestampForGoalZone(zone);

        /*
         * Goal pose must be selected before the turret creates
         * its one-loop sensor snapshot.
         */
        turret.updateZoneForGoalPose(zone);

        /*
         * Read encoder and Pinpoint data once and calculate:
         *
         * - Current encoder position
         * - Target field angle
         * - Turret-relative angle
         * - Base target tick
         */
        turret.updateSensorSnapshot();

        boolean aimEnabled =
                shooterState
                        == SHOOTERSTATE.FLYWHEEL_RUNNING
                        || shooterState
                        == SHOOTERSTATE.KICKER_EXTEND
                        || shooterState
                        == SHOOTERSTATE.SHOOT_READY
                        || shooterState
                        == SHOOTERSTATE.SEQUENCE_SHOOTING;

        /*
         * Query Limelight only while automatic aiming is active.
         */
        if (aimEnabled) {
            Limelight.TxSnapshot snapshot =
                    limelight.getTxForTag(24);

            setLimelightTx(
                    snapshot.hasTarget,
                    snapshot.txDeg
            );
        } else {
            setLimelightTx(
                    false,
                    Double.NaN
            );
        }

        int trimInput = 0;

        if (gamepadComboInput
                .getLbSinglePressedAny()) {
            trimInput += 1;
        }

        if (gamepadComboInput
                .getrbSinglePressedAny()) {
            trimInput -= 1;
        }

        if (turretState == TURRETSTATE.AIMING
                && aimEnabled) {

            trimTicks =
                    Range.clip(
                            trimTicks
                                    + trimInput
                                    * trimStep,
                            -400.0,
                            400.0
                    );

            int baseTargetTick =
                    turret.getTargetTick();

            int txAdjustTicks =
                    getTxAdjustTicks();

            int finalTargetTick =
                    baseTargetTick
                            + (int) Math.round(
                            trimTicks
                    )
                            + txAdjustTicks;

            /*
             * The updated Turret class uses its cached encoder
             * position internally.
             */
            turret.driveTurretPIDF(
                    finalTargetTick
            );
        } else {
            /*
             * Manual or locked control takes ownership of the
             * motor, so reset the profile before PIDF resumes.
             */
            turret.resetTurretProfile();

            robot.turretMotor.setVelocity(
                    trimInput * adjSpeed
            );
        }
    }

    // =========================================================
    // Turret-state toggle
    // =========================================================

    public void turretStateUpdate() {
        if (gamepadComboInput
                .getBothTriggersReleasedAny()) {

            turretState =
                    turretState
                            == TURRETSTATE.LOCKING
                            ? TURRETSTATE.AIMING
                            : TURRETSTATE.LOCKING;
        }
    }

    // =========================================================
    // Feed timing
    // =========================================================

    /**
     * Zones 1 through 5 use the close feed period.
     * Zones 6 and 7 use the far feed period.
     *
     * These config fields are read directly, so Dashboard
     * changes apply immediately.
     */
    public long getTimestampForGoalZone(int zone) {
        if (zone <= 5) {
            return Math.max(
                    0L,
                    FEED_PERIOD_MS_CLOSE
            );
        }

        return Math.max(
                0L,
                FEED_PERIOD_MS_FAR
        );
    }

    // =========================================================
    // Limelight TX filtering
    // =========================================================

    public void setLimelightTx(
            boolean hasTarget,
            double txDegrees
    ) {
        llHasTarget =
                hasTarget
                        && Double.isFinite(txDegrees);

        long now =
                System.currentTimeMillis();

        if (!llHasTarget) {
            return;
        }

        double safeAlpha =
                Range.clip(
                        txAlpha,
                        0.0,
                        1.0
                );

        if (!txInitialized) {
            filteredTx = txDegrees;
            txInitialized = true;
        } else {
            filteredTx =
                    safeAlpha * txDegrees
                            + (1.0 - safeAlpha)
                            * filteredTx;
        }

        lastValidTx =
                filteredTx;

        lastValidTimeMs =
                now;
    }

    private int getTxAdjustTicks() {
        long now =
                System.currentTimeMillis();

        double txToUse;

        if (llHasTarget) {
            txToUse =
                    filteredTx;
        } else {
            long lostTimeMs =
                    now - lastValidTimeMs;

            long safeHoldMs =
                    Math.max(0L, txHoldMs);

            long safeFadeMs =
                    Math.max(0L, txFadeMs);

            if (!txInitialized) {
                txToUse = 0.0;
            } else if (lostTimeMs <= safeHoldMs) {
                txToUse =
                        lastValidTx;
            } else if (safeFadeMs > 0L
                    && lostTimeMs
                    <= safeHoldMs + safeFadeMs) {

                double fadeProgress =
                        (lostTimeMs - safeHoldMs)
                                / (double) safeFadeMs;

                txToUse =
                        lastValidTx
                                * (1.0 - fadeProgress);
            } else {
                txToUse = 0.0;
            }
        }

        if (Math.abs(txToUse)
                < Math.max(0.0, txDeadbandDeg)) {
            txToUse = 0.0;
        }

        int adjustment =
                (int) Math.round(
                        -txToUse
                                * degToTicks
                );

        int safeMaximumTicks =
                Math.max(0, txMaxTicks);

        return Range.clip(
                adjustment,
                -safeMaximumTicks,
                safeMaximumTicks
        );
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

    // =========================================================
    // Public helpers
    // =========================================================

    public boolean canExit() {
        return shooterState
                == SHOOTERSTATE.SHOOTER_IDLE;
    }

    public void resetTrim() {
        trimTicks = 0.0;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getPower() {
        return power;
    }

    public double getTargetRPM() {
        return shooterPowerCalculator.getRPM();
    }

    public double getMeasuredRPM() {
        return shooterPowerCalculator.getMeasureRPM();
    }

    public long getWaitTimeMs() {
        return waitTimeMs;
    }

    public SHOOTERMOTORSTATE getShooterMotorState() {
        return shooterMotorState;
    }

    public TURRETSTATE getTurretState() {
        return turretState;
    }
}