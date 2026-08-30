package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class LiftSubsystem extends SubsystemBase {
    private final double Ticks_Per_Moto_Revolution = 145.1;
    private final double Pulley_Ratio = 2.0 / 3.0;
    private final double Actual_Ticks_Per_Revolution = Ticks_Per_Moto_Revolution * Pulley_Ratio; //96.7
    private final double Spool_Circumference_MM = 32 * Math.PI; //100.5MM
    private final double Ticks_Per_MM = Actual_Ticks_Per_Revolution / Spool_Circumference_MM; //0.96
    private final RobotHardware robot;
    private final PIDFController pidfLift;
    private double targetPositionUp = 200 * Ticks_Per_MM;
    private int targetPositionDown = 0;
    private double targetPosition = targetPositionDown;

    private final double POSITION_TOLERANCE_TICKS = 5.0;
    private final double VELOCITY_TOLERANCE_TICKS_PER_SEC = 10.0;
    private double kP = 0.015, kI = 0, kD = 0, kF = 0;

    public LiftSubsystem(RobotHardware robot) {
        this.robot = robot;
        this.pidfLift = new PIDFController(kP, kI, kD, kF);
        pidfLift.setTolerance(POSITION_TOLERANCE_TICKS, VELOCITY_TOLERANCE_TICKS_PER_SEC);
    }
    @Override
    public void periodic(){
        double currentTicks = robot.leftLiftMotor.getCurrentPosition();
        double power = pidfLift.calculate(currentTicks, targetPosition);

        robot.leftLiftMotor.setPower(power);
        robot.rightLiftMotor.setPower(power);
    }
    public void extendLift (){
        this.targetPosition = targetPositionUp;
    }
    public void lowerLift (){
        this.targetPosition = targetPositionDown;
    }
    public boolean isAtTarget () {
        return pidfLift.atSetPoint();
    }
    public void stop (){
        robot.leftLiftMotor.setPower(0);
        robot.rightLiftMotor.setPower(0);
    }
    public double getCurrentPositionTicks() {
        return robot.leftLiftMotor.getCurrentPosition();
    }

    public double getTargetPositionMM() {
        return targetPosition / Ticks_Per_MM;
    }
    public double getTargetTick () {
        return targetPosition;
    }

    public double getCurrentTick (){
        return getCurrentPositionTicks();
    }
    public double getCurrentPositionMM() {
        return getCurrentPositionTicks() / Ticks_Per_MM;
    }
    public double getErrorMM() {
        return getTargetPositionMM() - getCurrentPositionMM();
    }
    public double getErrorTick(){
        return getTargetTick() - getCurrentTick();
    }
}
