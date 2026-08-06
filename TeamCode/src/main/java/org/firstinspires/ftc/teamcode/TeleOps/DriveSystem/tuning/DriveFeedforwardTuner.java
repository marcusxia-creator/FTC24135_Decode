package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.tuning;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Measures drivetrain encoder velocity at several fixed motor powers.
 *
 * The test produces:
 *
 *      motor power versus ticks per second
 *
 * Use the measured values to estimate:
 *
 *      power = kS * sign(velocity) + kV * velocity
 *
 * Safety:
 * - Test in a clear area.
 * - Begin with low maximum power.
 * - Be ready to press B to stop immediately.
 *
 * Dashboard:
 * - Open FTC Dashboard.
 * - Select motorPower and measuredTicksPerSecondNormalized
 *   to compare their shapes on the same graph.
 *
 * The raw measuredTicksPerSecond value is also available.
 */
@Config
@TeleOp(
        name = "Drive Feedforward Tuner",
        group = "Drive Tuning"
)
public class DriveFeedforwardTuner extends LinearOpMode {

    /*
     * Change these names to match your robot configuration.
     */
    public static String LEFT_FRONT_NAME = "frontLeftMotor";
    public static String RIGHT_FRONT_NAME = "frontRightMotor";
    public static String LEFT_BACK_NAME = "backLeftMotor";
    public static String RIGHT_BACK_NAME = "backRightMotor";

    /**
     * Plotting
     */
    public static String REPORT_FOLDER_NAME =
            "DriveFeedforward";

    public static String REPORT_FILE_NAME =
            "feedforward_plot.html";

    private String generatedReportPath =
            "Not generated";

    /*
     * ============================================================
     * Test settings
     * ============================================================
     */

    /**
     * Lowest positive power included in the automatic test.
     *
     * Start low, but not so low that the drivetrain cannot move.
     */
    public static double MINIMUM_TEST_POWER = 0.10;

    /**
     * Highest positive power included in the automatic test.
     *
     * Start around 0.50 during the first test.
     */
    public static double MAXIMUM_TEST_POWER = 0.80;

    /**
     * Difference between consecutive test power levels.
     */
    public static double POWER_STEP = 0.10;

    /**
     * Time allowed for velocity to stabilize after changing power.
     */
    public static double SETTLE_TIME_SECONDS = 1.00;

    /**
     * Time used to average encoder velocity after settling.
     */
    public static double SAMPLE_TIME_SECONDS = 1.00;

    /**
     * Pause between test points.
     */
    public static double PAUSE_TIME_SECONDS = 0.30;

    /**
     * Test negative motor powers after positive powers.
     */
    public static boolean TEST_REVERSE = true;

    /**
     * Approximate maximum drivetrain velocity used only to normalize
     * velocity for the Dashboard time graph.
     *
     * This does not affect the measured result.
     */
    public static double GRAPH_VELOCITY_SCALE_TICKS_PER_SECOND = 2500.0;

    /**
     * Maximum number of power points that can be recorded.
     */
    private static final int MAXIMUM_RESULT_COUNT = 30;

    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;

    private final double[] recordedPower =
            new double[MAXIMUM_RESULT_COUNT];

    private final double[] recordedVelocity =
            new double[MAXIMUM_RESULT_COUNT];

    private int recordedResultCount = 0;

    private boolean previousAButton = false;

    private enum TestState {
        WAITING,
        SETTLING,
        SAMPLING,
        PAUSING,
        FINISHED,
        CANCELLED
    }

    private TestState testState = TestState.WAITING;

    private final ElapsedTime stateTimer =
            new ElapsedTime();

    private double currentTestPower = 0.0;

    private double accumulatedVelocity = 0.0;
    private int velocitySampleCount = 0;

    @Override
    public void runOpMode() {

        initializeMotors();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        stopDriveMotors();

        telemetry.addLine("Drive feedforward tuner ready.");
        telemetry.addLine("Press A to start.");
        telemetry.addLine("Press B to cancel.");
        telemetry.addLine("Start with the robot in a clear test area.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            boolean aButtonPressed =
                    gamepad1.a && !previousAButton;

            previousAButton = gamepad1.a;

            if (gamepad1.b) {
                cancelTest();
            }

            if (aButtonPressed
                    && (
                    testState == TestState.WAITING
                            || testState == TestState.FINISHED
                            || testState == TestState.CANCELLED
            )) {
                startTest();
            }

            updateTest();

            sendTelemetry();

            idle();
        }

        stopDriveMotors();
    }

    private void initializeMotors() {

        leftFrontMotor = hardwareMap.get(
                DcMotorEx.class,
                LEFT_FRONT_NAME
        );

        rightFrontMotor = hardwareMap.get(
                DcMotorEx.class,
                RIGHT_FRONT_NAME
        );

        leftBackMotor = hardwareMap.get(
                DcMotorEx.class,
                LEFT_BACK_NAME
        );

        rightBackMotor = hardwareMap.get(
                DcMotorEx.class,
                RIGHT_BACK_NAME
        );

        /*
         * These directions are common for mecanum drivetrains,
         * but verify them against your RobotHardware class.
         *
         * The important result is that positive power makes all
         * wheels drive the robot in the same forward direction.
         */
        leftFrontMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        leftBackMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        rightFrontMotor.setDirection(
                DcMotor.Direction.FORWARD
        );

        rightBackMotor.setDirection(
                DcMotor.Direction.FORWARD
        );

        /*
         * RUN_WITHOUT_ENCODER lets setPower() directly control
         * motor output while getVelocity() still reads the encoder.
         */
        leftFrontMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightFrontMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        leftBackMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        rightBackMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        leftFrontMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightFrontMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        leftBackMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightBackMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
    }

    private void startTest() {

        validateSettings();

        recordedResultCount = 0;

        currentTestPower =
                Math.abs(MINIMUM_TEST_POWER);

        accumulatedVelocity = 0.0;
        velocitySampleCount = 0;

        testState = TestState.SETTLING;

        stateTimer.reset();

        setDrivePower(currentTestPower);
    }

    private void updateTest() {

        switch (testState) {

            case WAITING:
            case FINISHED:
            case CANCELLED:
                stopDriveMotors();
                break;

            case SETTLING:
                updateSettlingState();
                break;

            case SAMPLING:
                updateSamplingState();
                break;

            case PAUSING:
                updatePausingState();
                break;
        }
    }

    private void updateSettlingState() {

        setDrivePower(currentTestPower);

        if (stateTimer.seconds()
                >= SETTLE_TIME_SECONDS) {

            accumulatedVelocity = 0.0;
            velocitySampleCount = 0;

            stateTimer.reset();
            testState = TestState.SAMPLING;
        }
    }

    private void updateSamplingState() {

        setDrivePower(currentTestPower);

        double averageVelocity =
                getAverageDriveVelocityTicksPerSecond();

        /*
         * Ignore invalid samples.
         */
        if (Double.isFinite(averageVelocity)) {
            accumulatedVelocity += averageVelocity;
            velocitySampleCount++;
        }

        if (stateTimer.seconds()
                >= SAMPLE_TIME_SECONDS) {

            double stableVelocity = 0.0;

            if (velocitySampleCount > 0) {
                stableVelocity =
                        accumulatedVelocity
                                / velocitySampleCount;
            }

            recordResult(
                    currentTestPower,
                    stableVelocity
            );

            stopDriveMotors();

            stateTimer.reset();
            testState = TestState.PAUSING;
        }
    }

    private void updatePausingState() {

        stopDriveMotors();

        if (stateTimer.seconds()
                < PAUSE_TIME_SECONDS) {
            return;
        }

        if (moveToNextPowerLevel()) {

            setDrivePower(currentTestPower);

            stateTimer.reset();
            testState = TestState.SETTLING;

        } else {

            finishTest();
        }
    }

    /**
     * Returns true when another power level is available.
     */
    private boolean moveToNextPowerLevel() {

        double powerStep =
                Math.abs(POWER_STEP);

        double maximumPower =
                Math.abs(MAXIMUM_TEST_POWER);

        /*
         * Positive-power test.
         */
        if (currentTestPower > 0.0) {

            double nextPositivePower =
                    currentTestPower + powerStep;

            if (nextPositivePower
                    <= maximumPower + 0.000001) {

                currentTestPower =
                        Math.min(
                                nextPositivePower,
                                maximumPower
                        );

                return true;
            }

            /*
             * Begin reverse testing.
             */
            if (TEST_REVERSE) {
                currentTestPower =
                        -Math.abs(MINIMUM_TEST_POWER);

                return true;
            }

            return false;
        }

        /*
         * Negative-power test.
         *
         * Example:
         * -0.10, -0.20, -0.30...
         */
        double nextNegativePower =
                currentTestPower - powerStep;

        if (Math.abs(nextNegativePower)
                <= maximumPower + 0.000001) {

            currentTestPower =
                    -Math.min(
                            Math.abs(nextNegativePower),
                            maximumPower
                    );

            return true;
        }

        return false;
    }

    private void recordResult(
            double motorPower,
            double velocityTicksPerSecond
    ) {

        if (recordedResultCount
                >= MAXIMUM_RESULT_COUNT) {
            return;
        }

        recordedPower[recordedResultCount] =
                motorPower;

        recordedVelocity[recordedResultCount] =
                velocityTicksPerSecond;

        recordedResultCount++;

        RobotLog.ii(
                "DriveFeedforwardTuner",
                "power=%.4f, velocityTicksPerSecond=%.2f",
                motorPower,
                velocityTicksPerSecond
        );
    }

    private void finishTest() {

        stopDriveMotors();

        testState = TestState.FINISHED;

        printResultsToLog();

        generateStaticPlotPage();
    }

    private void cancelTest() {

        stopDriveMotors();

        testState = TestState.CANCELLED;

        currentTestPower = 0.0;
    }

    private void setDrivePower(double power) {

        double safePower =
                clamp(power, -1.0, 1.0);

        leftFrontMotor.setPower(safePower);
        rightFrontMotor.setPower(safePower);
        leftBackMotor.setPower(safePower);
        rightBackMotor.setPower(safePower);
    }

    private void stopDriveMotors() {

        if (leftFrontMotor != null) {
            leftFrontMotor.setPower(0.0);
        }

        if (rightFrontMotor != null) {
            rightFrontMotor.setPower(0.0);
        }

        if (leftBackMotor != null) {
            leftBackMotor.setPower(0.0);
        }

        if (rightBackMotor != null) {
            rightBackMotor.setPower(0.0);
        }
    }

    /**
     * Returns the average signed drivetrain encoder velocity.
     *
     * Motor directions must be configured so positive forward drive
     * produces positive velocity from all four motors.
     */
    private double getAverageDriveVelocityTicksPerSecond() {

        double leftFrontVelocity =
                leftFrontMotor.getVelocity();

        double rightFrontVelocity =
                rightFrontMotor.getVelocity();

        double leftBackVelocity =
                leftBackMotor.getVelocity();

        double rightBackVelocity =
                rightBackMotor.getVelocity();

        return (
                leftFrontVelocity
                        + rightFrontVelocity
                        + leftBackVelocity
                        + rightBackVelocity
        ) / 4.0;
    }

    private void sendTelemetry() {

        double measuredVelocity =
                getAverageDriveVelocityTicksPerSecond();

        double normalizedVelocity = 0.0;

        if (GRAPH_VELOCITY_SCALE_TICKS_PER_SECOND
                > 0.0) {

            normalizedVelocity =
                    measuredVelocity
                            / GRAPH_VELOCITY_SCALE_TICKS_PER_SECOND;
        }

        telemetry.addData(
                "testState",
                testState
        );

        telemetry.addData(
                "motorPower",
                currentTestPower
        );

        telemetry.addData(
                "measuredTicksPerSecond",
                measuredVelocity
        );

        /*
         * Select these two values on the Dashboard graph:
         *
         * motorPower
         * measuredTicksPerSecondNormalized
         *
         * They are on approximately the same vertical scale.
         */
        telemetry.addData(
                "measuredTicksPerSecondNormalized",
                normalizedVelocity
        );

        telemetry.addData(
                "leftFrontTicksPerSecond",
                leftFrontMotor.getVelocity()
        );

        telemetry.addData(
                "rightFrontTicksPerSecond",
                rightFrontMotor.getVelocity()
        );

        telemetry.addData(
                "leftBackTicksPerSecond",
                leftBackMotor.getVelocity()
        );

        telemetry.addData(
                "rightBackTicksPerSecond",
                rightBackMotor.getVelocity()
        );

        telemetry.addData(
                "recordedPoints",
                recordedResultCount
        );

        telemetry.addData(
                "instructions",
                "A=start, B=cancel"
        );

        telemetry.addData(
                "staticReport",
                generatedReportPath
        );

        if (testState == TestState.FINISHED
                || testState == TestState.CANCELLED) {

            addRecordedResultsToTelemetry();
        }

        telemetry.update();
    }

    private void addRecordedResultsToTelemetry() {

        telemetry.addLine(
                "Power | Ticks/second"
        );

        for (int index = 0;
             index < recordedResultCount;
             index++) {

            telemetry.addData(
                    String.format(
                            "Point %02d",
                            index + 1
                    ),
                    "%.3f | %.1f",
                    recordedPower[index],
                    recordedVelocity[index]
            );
        }
    }

    private void printResultsToLog() {

        RobotLog.ii(
                "DriveFeedforwardTuner",
                "===== POWER VS VELOCITY RESULTS ====="
        );

        RobotLog.ii(
                "DriveFeedforwardTuner",
                "power,velocityTicksPerSecond"
        );

        for (int index = 0;
             index < recordedResultCount;
             index++) {

            RobotLog.ii(
                    "DriveFeedforwardTuner",
                    "%.5f,%.3f",
                    recordedPower[index],
                    recordedVelocity[index]
            );
        }

        FeedforwardFit fit =
                calculateLinearFeedforwardFit();

        RobotLog.ii(
                "DriveFeedforwardTuner",
                "Estimated kS = %.8f",
                fit.kS
        );

        RobotLog.ii(
                "DriveFeedforwardTuner",
                "Estimated kV = %.10f power per tick/second",
                fit.kV
        );

        RobotLog.ii(
                "DriveFeedforwardTuner",
                "R squared = %.5f",
                fit.rSquared
        );
    }

    /**
     * Fits:
     *
     *      power = kS * sign(velocity) + kV * velocity
     *
     * The test should include both positive and negative results for
     * the most useful estimate.
     */
    private FeedforwardFit calculateLinearFeedforwardFit() {

        if (recordedResultCount < 2) {
            return new FeedforwardFit(
                    0.0,
                    0.0,
                    0.0
            );
        }

        /*
         * Transform velocity so positive and negative test points can
         * be fitted together:
         *
         * absolutePower =
         *      kS + kV * absoluteVelocity
         */
        double sumVelocity = 0.0;
        double sumPower = 0.0;
        double sumVelocitySquared = 0.0;
        double sumVelocityPower = 0.0;

        int validCount = 0;

        for (int index = 0;
             index < recordedResultCount;
             index++) {

            double absoluteVelocity =
                    Math.abs(recordedVelocity[index]);

            double absolutePower =
                    Math.abs(recordedPower[index]);

            /*
             * Very low velocity points may represent a drivetrain
             * that has not overcome static friction.
             */
            if (absoluteVelocity < 1.0) {
                continue;
            }

            sumVelocity += absoluteVelocity;
            sumPower += absolutePower;

            sumVelocitySquared +=
                    absoluteVelocity * absoluteVelocity;

            sumVelocityPower +=
                    absoluteVelocity * absolutePower;

            validCount++;
        }

        if (validCount < 2) {
            return new FeedforwardFit(
                    0.0,
                    0.0,
                    0.0
            );
        }

        double denominator =
                validCount * sumVelocitySquared
                        - sumVelocity * sumVelocity;

        if (Math.abs(denominator) < 0.000000001) {
            return new FeedforwardFit(
                    0.0,
                    0.0,
                    0.0
            );
        }

        double kV =
                (
                        validCount * sumVelocityPower
                                - sumVelocity * sumPower
                ) / denominator;

        double kS =
                (
                        sumPower
                                - kV * sumVelocity
                ) / validCount;

        /*
         * Calculate R² as a simple indication of linear fit quality.
         */
        double meanPower =
                sumPower / validCount;

        double totalVariation = 0.0;
        double unexplainedVariation = 0.0;

        for (int index = 0;
             index < recordedResultCount;
             index++) {

            double absoluteVelocity =
                    Math.abs(recordedVelocity[index]);

            double absolutePower =
                    Math.abs(recordedPower[index]);

            if (absoluteVelocity < 1.0) {
                continue;
            }

            double predictedPower =
                    kS + kV * absoluteVelocity;

            double totalDifference =
                    absolutePower - meanPower;

            double residual =
                    absolutePower - predictedPower;

            totalVariation +=
                    totalDifference * totalDifference;

            unexplainedVariation +=
                    residual * residual;
        }

        double rSquared = 0.0;

        if (totalVariation > 0.000000001) {
            rSquared =
                    1.0
                            - unexplainedVariation
                            / totalVariation;
        }

        return new FeedforwardFit(
                kS,
                kV,
                rSquared
        );
    }

    private void validateSettings() {

        if (!Double.isFinite(MINIMUM_TEST_POWER)
                || !Double.isFinite(MAXIMUM_TEST_POWER)
                || !Double.isFinite(POWER_STEP)
                || MINIMUM_TEST_POWER <= 0.0
                || MAXIMUM_TEST_POWER
                < MINIMUM_TEST_POWER
                || MAXIMUM_TEST_POWER > 1.0
                || POWER_STEP <= 0.0) {

            throw new IllegalArgumentException(
                    "Invalid feedforward power-test settings."
            );
        }

        if (SETTLE_TIME_SECONDS < 0.0
                || SAMPLE_TIME_SECONDS <= 0.0
                || PAUSE_TIME_SECONDS < 0.0) {

            throw new IllegalArgumentException(
                    "Invalid test timing settings."
            );
        }
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

    private static final class FeedforwardFit {

        private final double kS;
        private final double kV;
        private final double rSquared;

        private FeedforwardFit(
                double kS,
                double kV,
                double rSquared
        ) {
            this.kS = kS;
            this.kV = kV;
            this.rSquared = rSquared;
        }
    }

    private void generateStaticPlotPage() {
        File reportFolder = new File(
                AppUtil.FIRST_FOLDER,
                REPORT_FOLDER_NAME
        );

        File reportFile = new File(
                reportFolder,
                REPORT_FILE_NAME
        );

        try {
            FeedforwardPlotGenerator.FeedforwardFit fit =
                    FeedforwardPlotGenerator.generateHtmlReport(
                            reportFile,
                            recordedPower,
                            recordedVelocity,
                            recordedResultCount
                    );

            generatedReportPath =
                    reportFile.getAbsolutePath();

            RobotLog.ii(
                    "DriveFeedforwardTuner",
                    "Static report generated: %s",
                    generatedReportPath
            );

            RobotLog.ii(
                    "DriveFeedforwardTuner",
                    "HTML fit: kS=%.8f, kV=%.10f, R2=%.5f",
                    fit.getKS(),
                    fit.getKV(),
                    fit.getRSquared()
            );

        } catch (IOException exception) {

            generatedReportPath =
                    "Report generation failed: "
                            + exception.getMessage();

            RobotLog.ee(
                    "DriveFeedforwardTuner",
                    exception,
                    "Unable to generate static feedforward plot."
            );
        }
    }
}