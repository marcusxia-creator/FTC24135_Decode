package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class TurretController {

    private final DcMotorEx turretMotor;
    private final PIDController positionPID;

    /*
     * PID gains
     */
    private double kP;
    private double kI;
    private double kD;

    /*
     * Feedforward gains
     *
     * output = kS * sign(velocity)
     *        + kV * velocity
     *        + kA * acceleration
     */
    private double kS;
    private double kV;
    private double kA;

    /*
     * Motion profile limits
     */
    private double maxVelocityTicksPerSecond;
    private double maxAccelerationTicksPerSecondSquared;

    /*
     * Turret limits
     */
    private int minimumTick;
    private int maximumTick;

    /*
     * Profile state
     */
    private double commandedPositionTicks;
    private double commandedVelocityTicksPerSecond;

    private int targetPositionTicks;

    private long previousTimeNs;

    private double previousPositionTicks;
    private double measuredVelocityTicksPerSecond;

    private double pidOutput;
    private double feedforwardOutput;
    private double motorOutput;

    public TurretController(
            DcMotorEx turretMotor,
            double kP,
            double kI,
            double kD,
            double kS,
            double kV,
            double kA,
            double maxVelocityTicksPerSecond,
            double maxAccelerationTicksPerSecondSquared,
            int minimumTick,
            int maximumTick
    ) {
        this.turretMotor = turretMotor;

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.kS = kS;
        this.kV = kV;
        this.kA = kA;

        this.maxVelocityTicksPerSecond = maxVelocityTicksPerSecond;
        this.maxAccelerationTicksPerSecondSquared =
                maxAccelerationTicksPerSecondSquared;

        this.minimumTick = minimumTick;
        this.maximumTick = maximumTick;

        positionPID = new PIDController(kP, kI, kD);

        configureMotor();

        double initialPosition = turretMotor.getCurrentPosition();

        commandedPositionTicks = initialPosition;
        previousPositionTicks = initialPosition;
        targetPositionTicks = (int) initialPosition;

        previousTimeNs = System.nanoTime();
    }

    private void configureMotor() {
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0.0);
    }

    public void update() {
        long currentTimeNs = System.nanoTime();

        double deltaTime =
                (currentTimeNs - previousTimeNs) / 1_000_000_000.0;

        previousTimeNs = currentTimeNs;

        if (deltaTime <= 0.0 || deltaTime > 0.1) {
            return;
        }

        /// currentPosition Tick, measured velocity, Target Position Tick
        double currentPositionTicks = turretMotor.getCurrentPosition();

        measuredVelocityTicksPerSecond =
                (currentPositionTicks - previousPositionTicks) / deltaTime;

        previousPositionTicks = currentPositionTicks;

        targetPositionTicks = Range.clip(
                targetPositionTicks,
                minimumTick,
                maximumTick
        );

        double previousCommandedVelocity =
                commandedVelocityTicksPerSecond;

        ///  Update Motion Profile
        updateMotionProfile(deltaTime);

        /// Commanded Acceleration
        double commandedAcceleration =
                (commandedVelocityTicksPerSecond -
                        previousCommandedVelocity) / deltaTime;

        positionPID.setPID(kP, kI, kD);

        /*
         * PID follows the motion-profile position instead of jumping
         * directly to the final target.
         */
        pidOutput = positionPID.calculate(
                currentPositionTicks,
                commandedPositionTicks
        );

        feedforwardOutput = calculateFeedforward(
                commandedVelocityTicksPerSecond,
                commandedAcceleration
        );

        ///  motor power
        motorOutput = pidOutput + feedforwardOutput;

        motorOutput = Range.clip(motorOutput, -1.0, 1.0);

        motorOutput = applySoftLimits(
                motorOutput,
                currentPositionTicks
        );

        turretMotor.setPower(motorOutput);
        /// End for Main Update
    }

    ///  HELPER FUNCTIONS - Motion Profile Update Based on Time.
    private void updateMotionProfile(double deltaTime) {
        double positionError =
                targetPositionTicks - commandedPositionTicks;
        /// End if already at target.
        if (Math.abs(positionError) < 0.5 && Math.abs(commandedVelocityTicksPerSecond) < 1.0) {
                commandedPositionTicks = targetPositionTicks;
                commandedVelocityTicksPerSecond = 0.0;
                return;
        }
        /// Direction Determination
        double direction = Math.signum(positionError);

        /*
         * Maximum velocity that still allows the turret to stop at
         * the target:
         *
         * v = sqrt(2 * acceleration * distance)
         */
        double maximumVelocityAllowedNow  =
                Math.sqrt(2.0 * maxAccelerationTicksPerSecondSquared * Math.abs(positionError));

        double desiredVelocity = direction * Math.min(maxVelocityTicksPerSecond,maximumVelocityAllowedNow);

        double maximumVelocityChange = maxAccelerationTicksPerSecondSquared * deltaTime;

        commandedVelocityTicksPerSecond =
                limitRateOfChange(
                        commandedVelocityTicksPerSecond,
                        desiredVelocity,
                        maximumVelocityChange
                );

        commandedPositionTicks += commandedVelocityTicksPerSecond * deltaTime;

        /*
         * Prevent the profile from moving beyond the final target.
         */
        if (direction > 0 && commandedPositionTicks > targetPositionTicks) {
            commandedPositionTicks = targetPositionTicks;
            commandedVelocityTicksPerSecond = 0.0;
        } else if (direction < 0 && commandedPositionTicks < targetPositionTicks) {
            commandedPositionTicks = targetPositionTicks;
            commandedVelocityTicksPerSecond = 0.0;
        }

        commandedPositionTicks = Range.clip(
                commandedPositionTicks,
                minimumTick,
                maximumTick
        );
    }

    /// HELPER FUNCTIONS - FEEDFORWARD
    private double calculateFeedforward(
            double velocityTicksPerSecond,
            double accelerationTicksPerSecondSquared
    ) {
        double staticOutput = 0.0;

        if (Math.abs(velocityTicksPerSecond) > 1.0) {
            staticOutput = kS * Math.signum(velocityTicksPerSecond);
        }

        return staticOutput
                + kV * velocityTicksPerSecond
                + kA * accelerationTicksPerSecondSquared;
    }

    private double applySoftLimits(
            double requestedPower,
            double currentPositionTicks
    ) {
        if (currentPositionTicks <= minimumTick &&
                requestedPower < 0.0) {
            return 0.0;
        }

        if (currentPositionTicks >= maximumTick &&
                requestedPower > 0.0) {
            return 0.0;
        }

        return requestedPower;
    }

    private double limitRateOfChange(
            double currentValue,
            double targetValue,
            double maximumChange
    ) {
        double difference = targetValue - currentValue;

        if (Math.abs(difference) <= maximumChange) {
            return targetValue;
        }

        return currentValue +
                Math.signum(difference) * maximumChange;
    }

    public void setTargetPositionTicks(int targetPositionTicks) {
        this.targetPositionTicks = Range.clip(
                targetPositionTicks,
                minimumTick,
                maximumTick
        );
    }

    public void addTargetTicks(int tickChange) {
        setTargetPositionTicks(targetPositionTicks + tickChange);
    }

    public void resetProfileToCurrentPosition() {
        double currentPosition = turretMotor.getCurrentPosition();

        commandedPositionTicks = currentPosition;
        commandedVelocityTicksPerSecond = 0.0;
        targetPositionTicks = (int) currentPosition;

        positionPID.reset();
    }

    public void stop() {
        turretMotor.setPower(0.0);
    }

    public boolean atTarget(
            double positionToleranceTicks,
            double velocityToleranceTicksPerSecond
    ) {
        double positionError =
                targetPositionTicks - turretMotor.getCurrentPosition();

        return Math.abs(positionError) <= positionToleranceTicks
                && Math.abs(measuredVelocityTicksPerSecond)
                <= velocityToleranceTicksPerSecond;
    }

    public void setPID(
            double kP,
            double kI,
            double kD
    ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        positionPID.setPID(kP, kI, kD);
    }

    public void setFeedforward(
            double kS,
            double kV,
            double kA
    ) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public void setMotionLimits(
            double maxVelocityTicksPerSecond,
            double maxAccelerationTicksPerSecondSquared
    ) {
        this.maxVelocityTicksPerSecond =
                Math.abs(maxVelocityTicksPerSecond);

        this.maxAccelerationTicksPerSecondSquared =
                Math.abs(maxAccelerationTicksPerSecondSquared);
    }

    public int getCurrentPositionTicks() {
        return turretMotor.getCurrentPosition();
    }

    public int getTargetPositionTicks() {
        return targetPositionTicks;
    }

    public double getCommandedPositionTicks() {
        return commandedPositionTicks;
    }

    public double getCommandedVelocityTicksPerSecond() {
        return commandedVelocityTicksPerSecond;
    }

    public double getMeasuredVelocityTicksPerSecond() {
        return measuredVelocityTicksPerSecond;
    }

    public double getPositionErrorTicks() {
        return commandedPositionTicks -
                turretMotor.getCurrentPosition();
    }

    public double getFinalTargetErrorTicks() {
        return targetPositionTicks -
                turretMotor.getCurrentPosition();
    }

    public double getPidOutput() {
        return pidOutput;
    }

    public double getFeedforwardOutput() {
        return feedforwardOutput;
    }

    public double getMotorOutput() {
        return motorOutput;
    }
}
