package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueCloseGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueFarGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redCloseGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redFarGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_X_Offset;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_Y_Offset;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class TurretUpd {

    private final RobotHardware robot;
    private final PIDController pidController;

    // =========================================================
    // Encoder and angle conversion
    // =========================================================

    private final double tickToAngle =
            0.16867469879518 * (360.0 / 145.1);

    private final double angleToTick =
            1.0 / tickToAngle;

    /*
     * Turret offsets are stored in metres.
     * Pinpoint position is requested in inches.
     */
    private static final double METERS_TO_INCHES =
            39.3700787;

    // =========================================================
    // Dashboard PID and feedforward tuning
    // =========================================================

    public static double kPTurret = 0.004;
    public static double kITurret = 0.0;
    public static double kDTurret = 0.0003;

    public static double kSTurret = 0.0001;
    public static double kVTurret = 0.0004;
    public static double kATurret = 0.0004;

    /*
     * Active PID values used to detect Dashboard changes.
     */
    private double activeKP;
    private double activeKI;
    private double activeKD;

    // =========================================================
    // Dashboard motion-profile tuning
    // =========================================================

    public static double maxVel = 1200.0;
    public static double maxAccel = 10000.0;

    public static double turretToleranceTicks = 5.0;

    public static double turretVelocityToleranceTicksPerSec =
            5.0;

    // =========================================================
    // Motion-profile state
    // =========================================================

    private double profilePositionTicks;
    private double profileVelocityTicksPerSecond;
    private double turretProfileAcceleration;

    private long previousTimeNs;
    private boolean turretProfileInitialized;

    // =========================================================
    // PIDF output values for telemetry
    // =========================================================

    private double turretPIDOutput;
    private double turretFeedforwardOutput;
    private double turretMotorOutput;

    // =========================================================
    // Cached one-loop hardware and pose snapshot
    // =========================================================

    private int cachedCurrentTick;

    private double cachedRobotXInches;
    private double cachedRobotYInches;

    private double cachedRobotHeadingRadians;
    private double cachedRobotHeadingDegrees;

    private double cachedTargetAngleDegrees;
    private double cachedTurretDriveAngleDegrees;

    private int cachedTargetTick;

    private boolean sensorSnapshotValid;

    // =========================================================
    // Turret geometry
    // =========================================================

    /*
     * atan2() correctly handles negative X offsets.
     */
    private final double turretOffsetTheta =
            Math.atan2(
                    turret_Center_Y_Offset,
                    turret_Center_X_Offset
            );

    private final double turretCenterOffsetLength =
            Math.hypot(
                    turret_Center_X_Offset,
                    turret_Center_Y_Offset
            );

    // =========================================================
    // Goal poses
    // =========================================================

    private final LUT<Integer, Pose2D> redTargetPose =
            new LUT<Integer, Pose2D>() {{
                add(1, redCloseGoalPose);
                add(2, redFarGoalPose);
            }};

    private final LUT<Integer, Pose2D> blueTargetPose =
            new LUT<Integer, Pose2D>() {{
                add(1, blueCloseGoalPose);
                add(2, blueFarGoalPose);
            }};

    private final LUT<Integer, Pose2D> targetPose;

    private Pose2D goalPose;

    // =========================================================
    // Turret reset
    // =========================================================

    private int turretOffsetTick;
    private double turretLSZeroTick;

    /*
     * Most REV digital limit switches are active-low:
     * false means pressed.
     *
     * Change this through Dashboard if your switch is active-high.
     */
    public static boolean limitSwitchActiveLow = true;

    // =========================================================
    // Constructor
    // =========================================================

    public TurretUpd(
            RobotHardware robot,
            boolean isRedAlliance
    ) {
        this.robot = robot;

        pidController = new PIDController(
                kPTurret,
                kITurret,
                kDTurret
        );

        pidController.setTolerance(
                turretToleranceTicks
        );

        activeKP = kPTurret;
        activeKI = kITurret;
        activeKD = kDTurret;

        targetPose =
                isRedAlliance
                        ? redTargetPose
                        : blueTargetPose;

        goalPose = targetPose.get(1);

        if (goalPose == null) {
            goalPose = targetPose.get(2);
        }

        previousTimeNs = System.nanoTime();
    }

    // =========================================================
    // One-loop sensor snapshot
    // =========================================================

    /**
     * Reads all information needed by the turret once.
     *
     * Call this exactly once per OpMode loop after:
     *
     * 1. Clearing the REV Hub bulk cache
     * 2. Updating the Pinpoint
     * 3. Selecting the turret goal pose
     */
    public void updateSensorSnapshot() {

        /*
         * Turret encoder read.
         *
         * With bulk caching enabled, this comes from the current
         * REV Hub bulk-data snapshot.
         */
        cachedCurrentTick =
                robot.turretMotor.getCurrentPosition();

        /*
         * Read Pinpoint pose once.
         */
        cachedRobotHeadingRadians =
                robot.pinpoint.getHeading(
                        AngleUnit.RADIANS
                );

        cachedRobotHeadingDegrees =
                Math.toDegrees(
                        cachedRobotHeadingRadians
                );

        cachedRobotXInches =
                robot.pinpoint.getPosX(
                        DistanceUnit.INCH
                );

        cachedRobotYInches =
                robot.pinpoint.getPosY(
                        DistanceUnit.INCH
                );

        /*
         * Calculate all turret targets once.
         */
        cachedTargetAngleDegrees =
                calculateTargetAngleFromSnapshot();

        cachedTurretDriveAngleDegrees =
                -wrapDegrees(
                        cachedRobotHeadingDegrees
                                - cachedTargetAngleDegrees
                );

        cachedTargetTick =
                (int) Math.round(
                        Range.clip(
                                cachedTurretDriveAngleDegrees,
                                -180.0,
                                180.0
                        ) * angleToTick
                );

        sensorSnapshotValid = true;
    }

    /**
     * Calculates the field angle from the turret center to the
     * selected goal using the cached Pinpoint pose.
     */
    private double calculateTargetAngleFromSnapshot() {
        if (goalPose == null) {
            return cachedRobotHeadingDegrees;
        }

        double turretOffsetFieldAngle =
                turretOffsetTheta
                        + cachedRobotHeadingRadians;

        double turretXOffsetInches =
                Math.cos(turretOffsetFieldAngle)
                        * turretCenterOffsetLength
                        * METERS_TO_INCHES;

        double turretYOffsetInches =
                Math.sin(turretOffsetFieldAngle)
                        * turretCenterOffsetLength
                        * METERS_TO_INCHES;

        double turretCenterX =
                cachedRobotXInches
                        + turretXOffsetInches;

        double turretCenterY =
                cachedRobotYInches
                        + turretYOffsetInches;

        double targetDeltaX =
                goalPose.getX(DistanceUnit.INCH)
                        - turretCenterX;

        double targetDeltaY =
                goalPose.getY(DistanceUnit.INCH)
                        - turretCenterY;

        return Math.toDegrees(
                Math.atan2(
                        targetDeltaY,
                        targetDeltaX
                )
        );
    }

    // =========================================================
    // Dashboard PID update
    // =========================================================

    /**
     * Updates the active FTCLib PID controller when Dashboard
     * PID values change.
     *
     * Feedforward values are read directly every loop.
     */
    private void updatePIDParameters() {
        boolean pidChanged =
                Double.compare(
                        activeKP,
                        kPTurret
                ) != 0
                        || Double.compare(
                        activeKI,
                        kITurret
                ) != 0
                        || Double.compare(
                        activeKD,
                        kDTurret
                ) != 0;

        if (pidChanged) {
            pidController.setPID(
                    kPTurret,
                    kITurret,
                    kDTurret
            );

            pidController.reset();

            activeKP = kPTurret;
            activeKI = kITurret;
            activeKD = kDTurret;
        }

        pidController.setTolerance(
                turretToleranceTicks
        );
    }

    // =========================================================
    // Motion-profile PIDF
    // =========================================================

    /**
     * Runs motion-profile PID plus feedforward.
     *
     * Uses cachedCurrentTick from updateSensorSnapshot().
     *
     * Call once per OpMode loop.
     */
    public void driveTurretPIDF(int targetTick) {
        if (!sensorSnapshotValid) {
            robot.turretMotor.setPower(0.0);
            return;
        }

        int currentTick =
                cachedCurrentTick;

        updatePIDParameters();

        long currentTimeNs =
                System.nanoTime();

        /*
         * Start the motion profile from the actual turret
         * position.
         */
        if (!turretProfileInitialized) {
            profilePositionTicks = currentTick;
            profileVelocityTicksPerSecond = 0.0;
            turretProfileAcceleration = 0.0;

            previousTimeNs = currentTimeNs;
            turretProfileInitialized = true;
        }

        double measuredDeltaTime =
                (currentTimeNs - previousTimeNs)
                        / 1_000_000_000.0;

        previousTimeNs = currentTimeNs;

        /*
         * Protect against startup timing and long loop pauses.
         */
        if (!Double.isFinite(measuredDeltaTime)
                || measuredDeltaTime <= 0.0
                || measuredDeltaTime > 0.1) {
            measuredDeltaTime = 0.02;
        }

        /*
         * Limit the profile timestep so a slow loop does not
         * create a large profile jump.
         */
        double profileDeltaTime =
                Math.min(
                        measuredDeltaTime,
                        0.05
                );

        double usableMaxVelocity =
                Math.max(0.0, maxVel);

        double usableMaxAcceleration =
                Math.max(0.0, maxAccel);

        double remainingDistanceTicks =
                targetTick
                        - profilePositionTicks;

        double direction =
                Math.signum(
                        remainingDistanceTicks
                );

        /*
         * Maximum velocity from which the profile can still
         * stop at the target:
         *
         * v = sqrt(2 × acceleration × distance)
         */
        double maximumVelocityAllowedNow =
                Math.sqrt(
                        2.0
                                * usableMaxAcceleration
                                * Math.abs(
                                remainingDistanceTicks
                        )
                );

        double desiredVelocity =
                direction
                        * Math.min(
                        usableMaxVelocity,
                        maximumVelocityAllowedNow
                );

        double maximumVelocityChange =
                usableMaxAcceleration
                        * profileDeltaTime;

        double previousProfileVelocity =
                profileVelocityTicksPerSecond;

        profileVelocityTicksPerSecond =
                applyAccelerationLimit(
                        profileVelocityTicksPerSecond,
                        desiredVelocity,
                        maximumVelocityChange
                );

        turretProfileAcceleration =
                (profileVelocityTicksPerSecond
                        - previousProfileVelocity)
                        / profileDeltaTime;

        double nextProfilePosition =
                profilePositionTicks
                        + profileVelocityTicksPerSecond
                        * profileDeltaTime;

        /*
         * Prevent the motion profile from crossing the target.
         */
        boolean profileCrossedTarget =
                (direction > 0.0
                        && nextProfilePosition >= targetTick)
                        || (direction < 0.0
                        && nextProfilePosition <= targetTick);

        if (profileCrossedTarget) {
            profilePositionTicks = targetTick;
            profileVelocityTicksPerSecond = 0.0;
            turretProfileAcceleration = 0.0;
        } else {
            profilePositionTicks =
                    nextProfilePosition;
        }

        /*
         * PID position correction.
         *
         * measurement = actual turret position
         * setpoint = generated profile position
         */
        turretPIDOutput =
                pidController.calculate(
                        currentTick,
                        profilePositionTicks
                );

        /*
         * Feedforward.
         */
        double staticFeedforward = 0.0;

        if (Math.abs(profileVelocityTicksPerSecond) > 2.0) {
            staticFeedforward =
                    kSTurret
                            * Math.signum(
                            profileVelocityTicksPerSecond
                    );
        }

        turretFeedforwardOutput =staticFeedforward
                + kVTurret*profileVelocityTicksPerSecond
                + kATurret*turretProfileAcceleration;

        turretMotorOutput =
                Range.clip(
                        turretPIDOutput
                                + turretFeedforwardOutput,
                        -1.0,
                        1.0
                );

        /*
         * Stop small remaining output when the turret and
         * motion profile have both reached the target.
         */
        boolean positionAtTarget =
                Math.abs(targetTick - currentTick)
                        <= turretToleranceTicks;

        boolean profileStopped =
                Math.abs(profileVelocityTicksPerSecond)
                        <= turretVelocityToleranceTicksPerSec;

        if (positionAtTarget && profileStopped) {
            profilePositionTicks = targetTick;
            profileVelocityTicksPerSecond = 0.0;
            turretProfileAcceleration = 0.0;

            /*
             * Use zero only if the turret does not require
             * holding power.
             */
            turretMotorOutput = 0.0;
        }

        robot.turretMotor.setPower(
                turretMotorOutput
        );
    }

    /**
     * Compatibility method for old code.
     *
     * The currentTick parameter is ignored because the turret
     * uses cachedCurrentTick.
     *
     * Remove this method after updating every call to:
     *
     * driveTurretPIDF(targetTick)
     */
    @Deprecated
    public void driveTurretPIDF(
            int currentTick,
            int targetTick
    ) {
        driveTurretPIDF(targetTick);
    }

    /**
     * Limits how quickly profile velocity can change.
     */
    private double applyAccelerationLimit(
            double currentVelocity,
            double desiredVelocity,
            double maximumChange
    ) {
        double safeMaximumChange =
                Math.max(0.0, maximumChange);

        double difference =
                desiredVelocity
                        - currentVelocity;

        if (Math.abs(difference)
                <= safeMaximumChange) {
            return desiredVelocity;
        }

        return currentVelocity
                + Math.signum(difference)
                * safeMaximumChange;
    }

    /**
     * Resets the profile when another controller has taken
     * control of the turret.
     */
    public void resetTurretProfile() {
        turretProfileInitialized = false;

        profileVelocityTicksPerSecond = 0.0;
        turretProfileAcceleration = 0.0;

        previousTimeNs = System.nanoTime();

        pidController.reset();
    }

    // =========================================================
    // Simple PID controller
    // =========================================================

    public void driveTurretPID(int targetTick) {
        if (!sensorSnapshotValid) {
            robot.turretMotor.setPower(0.0);
            return;
        }

        int currentTick =
                cachedCurrentTick;

        updatePIDParameters();

        int errorTicks =
                targetTick - currentTick;

        double staticFeedforward = 0.0;

        if (Math.abs(errorTicks)
                > turretToleranceTicks) {
            staticFeedforward =
                    kSTurret
                            * Math.signum(errorTicks);
        }

        double pidOutput =
                pidController.calculate(
                        currentTick,
                        targetTick
                );

        double output =
                pidOutput
                        + staticFeedforward;

        robot.turretMotor.setPower(
                Range.clip(
                        output,
                        -1.0,
                        1.0
                )
        );
    }

    /**
     * Compatibility overload for old code.
     */
    @Deprecated
    public void driveTurretPID(
            int currentTick,
            int targetTick
    ) {
        driveTurretPID(targetTick);
    }

    // =========================================================
    // RUN_TO_POSITION test method
    // =========================================================

    public void driveTurretMotor() {
        if (!sensorSnapshotValid) {
            robot.turretMotor.setPower(0.0);
            return;
        }

        robot.turretMotor.setTargetPosition(
                cachedTargetTick
        );

        robot.turretMotor.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        robot.turretMotor.setPower(1.0);

        resetTurretProfile();
    }

    /**
     * Call once when changing from RUN_TO_POSITION back to PIDF.
     * Do not call this every loop.
     */
    public void prepareForPIDFControl() {
        robot.turretMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        resetTurretProfile();
    }

    // =========================================================
    // Goal selection
    // =========================================================

    /**
     * Zones 1 through 5 use the close target.
     * Zones 6 and 7 use the far target.
     *
     * Call before updateSensorSnapshot().
     */
    public void updateZoneForGoalPose(int zone) {
        int normalizedZone =
                zone <= 5 ? 1 : 2;

        Pose2D selectedGoal =
                targetPose.get(normalizedZone);

        if (selectedGoal == null) {
            selectedGoal =
                    targetPose.get(1);
        }

        goalPose = selectedGoal;

        /*
         * The target geometry must now be recalculated.
         */
        sensorSnapshotValid = false;
    }

    // =========================================================
    // Turret reset and limit switch
    // =========================================================

    public boolean turretReset(int startingTick) {
        /*
         * Prefer the cached value when a snapshot is available.
         */
        int currentTick =
                sensorSnapshotValid
                        ? cachedCurrentTick
                        : robot.turretMotor.getCurrentPosition();

        if (isLimitPressed()) {
            robot.turretMotor.setPower(0.0);

            robot.turretMotor.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER
            );

            robot.turretMotor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );

            cachedCurrentTick = 0;
            turretLSZeroTick = 0.0;

            sensorSnapshotValid = false;

            resetTurretProfile();

            return true;
        }

        int delta =
                currentTick
                        - startingTick;

        boolean nearStart =
                Math.abs(delta) < 435;

        double baseDirection =
                startingTick < 0
                        ? 1.0
                        : -1.0;

        turretOffsetTick =
                (int) Math.round(
                        baseDirection * 18.0
                );

        double searchDirection =
                nearStart
                        ? baseDirection
                        : -baseDirection;

        robot.turretMotor.setPower(
                0.5 * searchDirection
        );

        resetTurretProfile();

        return false;
    }

    public boolean isLimitPressed() {
        boolean switchState =
                robot.limitSwitch.getState();

        return limitSwitchActiveLow
                ? !switchState
                : switchState;
    }

    // =========================================================
    // Angle helpers
    // =========================================================

    /**
     * Wraps angle to the range [-180, 180).
     */
    private double wrapDegrees(
            double angleDegrees
    ) {
        return floorMod(
                angleDegrees + 180.0,
                360.0
        ) - 180.0;
    }

    private double floorMod(
            double value,
            double modulus
    ) {
        return value
                - Math.floor(value / modulus)
                * modulus;
    }

    // =========================================================
    // Cached-value getters
    // =========================================================

    /**
     * Returns the cached encoder position.
     * It does not perform another motor read.
     */
    public int getCurrentTick() {
        return cachedCurrentTick;
    }

    /**
     * Returns the cached target encoder position.
     */
    public int getTargetTick() {
        return cachedTargetTick;
    }

    public int getMotorDriveTick() {
        return cachedTargetTick;
    }

    /**
     * Returns the cached field-relative target angle.
     */
    public double getTargetAngle() {
        return cachedTargetAngleDegrees;
    }

    /**
     * Returns the cached turret-relative drive angle.
     */
    public double getTurretDriveAngle() {
        return cachedTurretDriveAngleDegrees;
    }

    public double getTurretMotorAngle() {
        return cachedCurrentTick
                * tickToAngle;
    }

    public Pose2D getGoalPose() {
        return goalPose;
    }

    public boolean isSensorSnapshotValid() {
        return sensorSnapshotValid;
    }

    // =========================================================
    // Motion-profile telemetry getters
    // =========================================================

    public double getProfilePositionTicks() {
        return profilePositionTicks;
    }

    public double getProfileVelocityTicksPerSecond() {
        return profileVelocityTicksPerSecond;
    }

    public double getProfileAcceleration() {
        return turretProfileAcceleration;
    }

    public double getPIDOutput() {
        return turretPIDOutput;
    }

    public double getFeedforwardOutput() {
        return turretFeedforwardOutput;
    }

    public double getMotorOutput() {
        return turretMotorOutput;
    }

    public int getTurretOffsetTick() {
        return turretOffsetTick;
    }

    public double getTurretLimitSwitchZeroTick() {
        return turretLSZeroTick;
    }

    public double getCachedRobotXInches() {
        return cachedRobotXInches;
    }

    public double getCachedRobotYInches() {
        return cachedRobotYInches;
    }

    public double getCachedRobotHeadingDegrees() {
        return cachedRobotHeadingDegrees;
    }
}