package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem;

/**
 * Motion Controller for DriveTrain
 * DriveSystem/
 * │
 * ├── DriveMotionController.java      <-- Main controller
 * │
 * ├── Path/
 * │   ├── DrivePath.java
 * │   ├── DrivePathState.java
 * │   ├── StraightLinePath.java
 * │   ├── CubicBezierPath.java
 * │   └── MultiSegmentPath.java
 * │
 * ├── Profile/
 * │   ├── MotionProfile.java
 * │   ├── MotionState.java
 * │   ├── TrapezoidalProfile.java
 * │   └── SCurveProfile.java
 * │
 * ├── PID/
 * │   ├── PIDController1D.java
 * │   ├── HeadingPIDController.java
 * │   └── FeedforwardCalculator.java
 * │
 * └── Utilities/
 *     ├── MotionMath.java
 *     └── AngleUtil.java
 */

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Feedforward.FeedforwardCalculator;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.DrivePath;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.DrivePathState;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.StraightLinePath;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile.MotionProfile;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile.MotionState;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile.TrapezoidalProfile;

/**
 * Controls a mecanum drivetrain along a DrivePath using:
 *
 * - time-based motion profile
 * - translation feedforward
 * - X and Y PID tracking
 * - heading PID
 *
 * Internal units:
 *
 * position: mm
 * velocity: mm/s
 * acceleration: mm/s²
 * heading: radians
 * time: seconds
 *
 * This class calculates motor powers but does not directly access
 * drivetrain motors. The TeleOp reads the four output powers and
 * sends them to RobotHardware.
 */
@Config
public final class DriveMotionController {

    /*
     * ============================================================
     * Translation motion-profile limits
     * ============================================================
     */

    public static double maximumVelocityMMPerSecond =
            1200.0;

    public static double maximumAccelerationMMPerSecondSquared =
            1800.0;

    public static double maximumDecelerationMMPerSecondSquared =
            1400.0;

    /*
     * ============================================================
     * Translation PID
     * ============================================================
     */

    public static double translationKP = 0.006;
    public static double translationKI = 0.0;
    public static double translationKD = 0.00015;

    /*
     * Limit the PID correction independently from feedforward.
     */
    public static double maximumTranslationPIDOutput =
            0.45;

    /*
     * ============================================================
     * Heading PID
     * ============================================================
     */

    public static double headingKP = 2.5;
    public static double headingKI = 0.0;
    public static double headingKD = 0.08;

    public static double maximumHeadingOutput =
            0.60;

    /*
     * ============================================================
     * Translation feedforward
     * ============================================================
     *
     * Initial estimate:
     *
     * kV = 1 / measured maximum velocity in mm/s
     */

    public static double translationKS = 0.06;
    public static double translationKV =
            1.0 / 1500.0;

    public static double translationKA = 0.0;

    /*
     * ============================================================
     * Completion settings
     * ============================================================
     */

    public static double positionToleranceMM = 15.0;

    public static double headingToleranceRadians =
            Math.toRadians(1.5);

    public static double settleTimeSeconds = 0.20;

    /*
     * Optional safety timeout.
     *
     * Set to zero to disable.
     */
    public static double timeoutSeconds = 5.0;

    /*
     * Maximum final drivetrain output.
     */
    public static double maximumDrivePower = 1.0;

    /*
     * ============================================================
     * FTCLib PID controllers
     * ============================================================
     */

    private final PIDController xPID =
            new PIDController(
                    translationKP,
                    translationKI,
                    translationKD
            );

    private final PIDController yPID =
            new PIDController(
                    translationKP,
                    translationKI,
                    translationKD
            );

    private final PIDController headingPID =
            new PIDController(
                    headingKP,
                    headingKI,
                    headingKD
            );

    private final FeedforwardCalculator translationFeedforward =
            new FeedforwardCalculator(
                    translationKS,
                    translationKV,
                    translationKA
            );

    /*
     * ============================================================
     * Active path and profile
     * ============================================================
     */

    private DrivePath activePath;
    private MotionProfile activeMotionProfile;

    private Pose2D targetPose;

    /*
     * ============================================================
     * Timers and status
     * ============================================================
     */

    private final ElapsedTime profileTimer =
            new ElapsedTime();

    private final ElapsedTime settleTimer =
            new ElapsedTime();

    private boolean active = false;
    private boolean settling = false;
    private boolean completedSuccessfully = false;
    private boolean cancelled = false;
    private boolean timedOut = false;

    /*
     * ============================================================
     * Mecanum outputs
     * ============================================================
     */

    private double leftFrontPower = 0.0;
    private double rightFrontPower = 0.0;
    private double leftBackPower = 0.0;
    private double rightBackPower = 0.0;

    /*
     * ============================================================
     * Telemetry values
     * ============================================================
     */

    private double plannedXMM = 0.0;
    private double plannedYMM = 0.0;

    private double plannedDistanceMM = 0.0;
    private double plannedVelocityMMPerSecond = 0.0;
    private double plannedAccelerationMMPerSecondSquared = 0.0;

    private double xTrackingErrorMM = 0.0;
    private double yTrackingErrorMM = 0.0;

    private double finalPositionErrorMM = 0.0;
    private double finalHeadingErrorRadians = 0.0;

    private double xFeedforwardOutput = 0.0;
    private double yFeedforwardOutput = 0.0;

    private double xPIDOutput = 0.0;
    private double yPIDOutput = 0.0;
    private double headingPIDOutput = 0.0;

    /**
     * Starts a straight-line movement from the current pose to the
     * target pose.
     *
     * Call once when the driver's semi-auto start button is pressed.
     */
    public void startStraightLine(
            Pose2D currentPose,
            Pose2D targetPose
    ) {
        if (currentPose == null) {
            throw new IllegalArgumentException(
                    "Current pose cannot be null."
            );
        }

        if (targetPose == null) {
            throw new IllegalArgumentException(
                    "Target pose cannot be null."
            );
        }

        DrivePath path =
                new StraightLinePath(
                        currentPose,
                        targetPose
                );

        MotionProfile profile =
                new TrapezoidalProfile(
                        path.getLengthMM(),
                        maximumVelocityMMPerSecond,
                        maximumAccelerationMMPerSecondSquared,
                        maximumDecelerationMMPerSecondSquared
                );

        start(
                targetPose,
                path,
                profile
        );
    }

    /**
     * Starts a movement using any DrivePath and MotionProfile.
     *
     * This method will later allow CubicBezierPath without changing
     * the controller.
     */
    public void start(
            Pose2D targetPose,
            DrivePath drivePath,
            MotionProfile motionProfile
    ) {
        if (targetPose == null) {
            throw new IllegalArgumentException(
                    "Target pose cannot be null."
            );
        }

        if (drivePath == null) {
            throw new IllegalArgumentException(
                    "DrivePath cannot be null."
            );
        }

        if (motionProfile == null) {
            throw new IllegalArgumentException(
                    "MotionProfile cannot be null."
            );
        }

        this.targetPose = targetPose;
        this.activePath = drivePath;
        this.activeMotionProfile = motionProfile;

        updatePIDCoefficients();
        updateFeedforwardCoefficients();

        xPID.reset();
        yPID.reset();
        headingPID.reset();

        profileTimer.reset();
        settleTimer.reset();

        active = true;
        settling = false;
        completedSuccessfully = false;
        cancelled = false;
        timedOut = false;

        setMotorOutputsToZero();
    }

    /**
     * Performs one controller update.
     *
     * Call once every TeleOp loop while isActive() is true.
     */
    public void update(
            Pose2D currentPose
    ) {
        if (!active || currentPose == null) {
            return;
        }

        updatePIDCoefficients();
        updateFeedforwardCoefficients();

        double elapsedTimeSeconds =
                profileTimer.seconds();

        /*
         * Ask the motion profile how far along the path the robot
         * should be at this exact elapsed time.
         */
        MotionState motionState =
                activeMotionProfile.getMotionState(
                        elapsedTimeSeconds
                );

        /*
         * Ask the active path where that distance is located on
         * the field.
         */
        DrivePathState pathState =
                activePath.getPathState(
                        motionState.getDistanceAlongPathMM()
                );

        plannedDistanceMM =
                motionState.getDistanceAlongPathMM();

        plannedVelocityMMPerSecond =
                motionState.getVelocityMMPerSecond();

        plannedAccelerationMMPerSecondSquared =
                motionState
                        .getAccelerationMMPerSecondSquared();

        plannedXMM = pathState.getXMM();
        plannedYMM = pathState.getYMM();

        double currentXMM =
                currentPose.getX(DistanceUnit.MM);

        double currentYMM =
                currentPose.getY(DistanceUnit.MM);

        double currentHeadingRadians =
                currentPose.getHeading(
                        AngleUnit.RADIANS
                );

        /*
         * ========================================================
         * Position tracking errors
         * ========================================================
         */

        xTrackingErrorMM =
                plannedXMM - currentXMM;

        yTrackingErrorMM =
                plannedYMM - currentYMM;

        /*
         * FTCLib calculate(measurement, setPoint).
         */
        xPIDOutput = xPID.calculate(
                currentXMM,
                plannedXMM
        );

        yPIDOutput = yPID.calculate(
                currentYMM,
                plannedYMM
        );

        xPIDOutput = clamp(
                xPIDOutput,
                -maximumTranslationPIDOutput,
                maximumTranslationPIDOutput
        );

        yPIDOutput = clamp(
                yPIDOutput,
                -maximumTranslationPIDOutput,
                maximumTranslationPIDOutput
        );

        /*
         * ========================================================
         * Translation feedforward
         * ========================================================
         */

        double pathFeedforwardOutput =
                translationFeedforward.calculate(
                        plannedVelocityMMPerSecond,
                        plannedAccelerationMMPerSecondSquared
                );

        /*
         * The scalar feedforward acts along the path tangent.
         */
        xFeedforwardOutput =
                pathState.getTangentX()
                        * pathFeedforwardOutput;

        yFeedforwardOutput =
                pathState.getTangentY()
                        * pathFeedforwardOutput;

        double xFieldCommand =
                xFeedforwardOutput + xPIDOutput;

        double yFieldCommand =
                yFeedforwardOutput + yPIDOutput;

        /*
         * ========================================================
         * Heading control
         * ========================================================
         *
         * In this first phase, the robot tracks the final requested
         * heading during the whole translation.
         *
         * A separate heading motion profile can be added later.
         */

        double targetHeadingRadians =
                targetPose.getHeading(
                        AngleUnit.RADIANS
                );

        finalHeadingErrorRadians =
                normalizeRadians(
                        targetHeadingRadians
                                - currentHeadingRadians
                );

        /*
         * Passing measurement = 0 and setpoint = wrapped error makes
         * FTCLib produce:
         *
         * PID error = wrappedHeadingError - 0
         */
        headingPIDOutput =
                headingPID.calculate(
                        0.0,
                        finalHeadingErrorRadians
                );

        headingPIDOutput = clamp(
                headingPIDOutput,
                -maximumHeadingOutput,
                maximumHeadingOutput
        );

        /*
         * ========================================================
         * Field frame to robot frame
         * ========================================================
         */

        double cosine =
                Math.cos(currentHeadingRadians);

        double sine =
                Math.sin(currentHeadingRadians);

        double robotForwardCommand =
                xFieldCommand * cosine
                        + yFieldCommand * sine;

        double robotStrafeCommand =
                -xFieldCommand * sine
                        + yFieldCommand * cosine;

        calculateMecanumPowers(
                robotForwardCommand,
                robotStrafeCommand,
                headingPIDOutput
        );

        updateCompletionState(
                currentXMM,
                currentYMM,
                targetHeadingRadians,
                currentHeadingRadians,
                elapsedTimeSeconds
        );
    }

    /**
     * Immediately ends semi-auto control and sets all outputs to zero.
     */
    public void cancel() {
        if (!active) {
            return;
        }

        active = false;
        settling = false;
        cancelled = true;
        completedSuccessfully = false;

        xPID.reset();
        yPID.reset();
        headingPID.reset();

        setMotorOutputsToZero();
    }

    private void updateCompletionState(
            double currentXMM,
            double currentYMM,
            double targetHeadingRadians,
            double currentHeadingRadians,
            double elapsedTimeSeconds
    ) {
        double targetXMM =
                targetPose.getX(DistanceUnit.MM);

        double targetYMM =
                targetPose.getY(DistanceUnit.MM);

        double finalXErrorMM =
                targetXMM - currentXMM;

        double finalYErrorMM =
                targetYMM - currentYMM;

        finalPositionErrorMM =
                Math.sqrt(
                        finalXErrorMM * finalXErrorMM
                                + finalYErrorMM
                                * finalYErrorMM
                );

        finalHeadingErrorRadians =
                normalizeRadians(
                        targetHeadingRadians
                                - currentHeadingRadians
                );

        boolean profileFinished =
                activeMotionProfile.isFinished(
                        elapsedTimeSeconds
                );

        boolean positionWithinTolerance =
                finalPositionErrorMM
                        <= positionToleranceMM;

        boolean headingWithinTolerance =
                Math.abs(finalHeadingErrorRadians)
                        <= headingToleranceRadians;

        boolean targetSatisfied =
                profileFinished
                        && positionWithinTolerance
                        && headingWithinTolerance;

        if (targetSatisfied) {
            if (!settling) {
                settleTimer.reset();
                settling = true;
            }

            if (settleTimer.seconds()
                    >= settleTimeSeconds) {

                finishSuccessfully();
                return;
            }
        } else {
            settling = false;
            settleTimer.reset();
        }

        if (timeoutSeconds > 0.0
                && elapsedTimeSeconds
                >= timeoutSeconds) {

            timedOut = true;
            active = false;
            settling = false;
            completedSuccessfully = false;

            setMotorOutputsToZero();
        }
    }

    private void finishSuccessfully() {
        active = false;
        settling = false;
        completedSuccessfully = true;
        cancelled = false;
        timedOut = false;

        setMotorOutputsToZero();
    }

    private void calculateMecanumPowers(
            double forward,
            double strafe,
            double turn
    ) {
        double calculatedLeftFront =
                forward + strafe + turn;

        double calculatedRightFront =
                forward - strafe - turn;

        double calculatedLeftBack =
                forward - strafe + turn;

        double calculatedRightBack =
                forward + strafe - turn;

        double largestMagnitude = Math.max(
                1.0,
                Math.max(
                        Math.abs(calculatedLeftFront),
                        Math.max(
                                Math.abs(calculatedRightFront),
                                Math.max(
                                        Math.abs(calculatedLeftBack),
                                        Math.abs(calculatedRightBack)
                                )
                        )
                )
        );

        double scale =
                maximumDrivePower
                        / largestMagnitude;

        leftFrontPower = clamp(
                calculatedLeftFront * scale,
                -1.0,
                1.0
        );

        rightFrontPower = clamp(
                calculatedRightFront * scale,
                -1.0,
                1.0
        );

        leftBackPower = clamp(
                calculatedLeftBack * scale,
                -1.0,
                1.0
        );

        rightBackPower = clamp(
                calculatedRightBack * scale,
                -1.0,
                1.0
        );
    }

    /**
     * Allows FTC Dashboard gain changes to take effect while running.
     */
    private void updatePIDCoefficients() {
        xPID.setPID(
                translationKP,
                translationKI,
                translationKD
        );

        yPID.setPID(
                translationKP,
                translationKI,
                translationKD
        );

        headingPID.setPID(
                headingKP,
                headingKI,
                headingKD
        );

        xPID.setTolerance(positionToleranceMM);
        yPID.setTolerance(positionToleranceMM);

        headingPID.setTolerance(
                headingToleranceRadians
        );
    }

    private void updateFeedforwardCoefficients() {
        translationFeedforward.setCoefficients(
                translationKS,
                translationKV,
                translationKA
        );
    }

    private void setMotorOutputsToZero() {
        leftFrontPower = 0.0;
        rightFrontPower = 0.0;
        leftBackPower = 0.0;
        rightBackPower = 0.0;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isFinishedSuccessfully() {
        return completedSuccessfully;
    }

    public boolean wasCancelled() {
        return cancelled;
    }

    public boolean hasTimedOut() {
        return timedOut;
    }

    public boolean isSettling() {
        return settling;
    }

    public double getLeftFrontPower() {
        return leftFrontPower;
    }

    public double getRightFrontPower() {
        return rightFrontPower;
    }

    public double getLeftBackPower() {
        return leftBackPower;
    }

    public double getRightBackPower() {
        return rightBackPower;
    }

    public double getPlannedXMM() {
        return plannedXMM;
    }

    public double getPlannedYMM() {
        return plannedYMM;
    }

    public double getPlannedDistanceMM() {
        return plannedDistanceMM;
    }

    public double getPlannedVelocityMMPerSecond() {
        return plannedVelocityMMPerSecond;
    }

    public double getPlannedAccelerationMMPerSecondSquared() {
        return plannedAccelerationMMPerSecondSquared;
    }

    public double getXTrackingErrorMM() {
        return xTrackingErrorMM;
    }

    public double getYTrackingErrorMM() {
        return yTrackingErrorMM;
    }

    public double getFinalPositionErrorMM() {
        return finalPositionErrorMM;
    }

    public double getFinalHeadingErrorRadians() {
        return finalHeadingErrorRadians;
    }

    public double getXFeedforwardOutput() {
        return xFeedforwardOutput;
    }

    public double getYFeedforwardOutput() {
        return yFeedforwardOutput;
    }

    public double getXPIDOutput() {
        return xPIDOutput;
    }

    public double getYPIDOutput() {
        return yPIDOutput;
    }

    public double getHeadingPIDOutput() {
        return headingPIDOutput;
    }

    public double getProfileElapsedTimeSeconds() {
        return profileTimer.seconds();
    }

    public double getProfileTotalTimeSeconds() {
        if (activeMotionProfile == null) {
            return 0.0;
        }

        return activeMotionProfile
                .getTotalTimeSeconds();
    }

    private static double normalizeRadians(
            double angleRadians
    ) {
        while (angleRadians > Math.PI) {
            angleRadians -=
                    2.0 * Math.PI;
        }

        while (angleRadians < -Math.PI) {
            angleRadians +=
                    2.0 * Math.PI;
        }

        return angleRadians;
    }

    private static double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}