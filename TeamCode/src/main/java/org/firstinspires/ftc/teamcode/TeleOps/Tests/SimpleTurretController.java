package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class SimpleTurretController {

    private final DcMotorEx turretMotor;
    private final PIDController pidController;

    private double kP;
    private double kI;
    private double kD;

    /*
     * Minimum power needed to overcome turret friction.
     */
    private double kS;

    private int targetTicks;

    private final int minimumTicks;
    private final int maximumTicks;

    private double positionError;
    private double pidOutput;
    private double feedforwardOutput;
    private double motorOutput;

    public SimpleTurretController(
            DcMotorEx turretMotor,
            double kP,
            double kI,
            double kD,
            double kS,
            int minimumTicks,
            int maximumTicks
    ) {
        this.turretMotor = turretMotor;

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;

        this.minimumTicks = minimumTicks;
        this.maximumTicks = maximumTicks;

        pidController = new PIDController(kP, kI, kD);

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetTicks = turretMotor.getCurrentPosition();
    }

    public void update() {

        int currentTicks = turretMotor.getCurrentPosition();

        targetTicks = Range.clip(
                targetTicks,
                minimumTicks,
                maximumTicks
        );

        positionError = targetTicks - currentTicks;

        /*
         * FTCLib calculates:
         *
         * error = target - current
         */
        pidController.setPID(kP, kI, kD);

        pidOutput = pidController.calculate(
                currentTicks,
                targetTicks
        );

        /*
         * Static-friction feedforward.
         *
         * Push in the direction of the position error.
         */
        if (Math.abs(positionError) > 5) {
            feedforwardOutput = kS * Math.signum(positionError);
        } else {
            feedforwardOutput = 0.0;
        }

        motorOutput = pidOutput + feedforwardOutput;

        motorOutput = Range.clip(
                motorOutput,
                -1.0,
                1.0
        );

        /*
         * Prevent movement beyond the soft limits.
         */
        if (currentTicks <= minimumTicks && motorOutput < 0.0) {
            motorOutput = 0.0;
        }

        if (currentTicks >= maximumTicks && motorOutput > 0.0) {
            motorOutput = 0.0;
        }

        turretMotor.setPower(motorOutput);
    }

    public void setTargetTicks(int targetTicks) {
        this.targetTicks = Range.clip(
                targetTicks,
                minimumTicks,
                maximumTicks
        );
    }

    public void addTargetTicks(int amount) {
        setTargetTicks(targetTicks + amount);
    }

    public void setPID(
            double kP,
            double kI,
            double kD
    ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setKS(double kS) {
        this.kS = kS;
    }

    public int getCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public double getPositionError() {
        return positionError;
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

    public boolean atTarget(int toleranceTicks) {
        return Math.abs(positionError) <= toleranceTicks;
    }

    public void stop() {
        turretMotor.setPower(0.0);
    }
}
