package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

public class IntakeSubsystem extends SubsystemBase{
    private RobotHardware robot;
    private double intakePower = 0.7;
    private final double POWER_STEP = 0.1;
    public IntakeSubsystem (RobotHardware robot) {
        this.robot = robot;
    }
    public void runRollers (){
        robot.intakeMotor.setPower(intakePower);
        robot.leftSideRoller.setPower(0.5);
        robot.rightSideRoller.setPower(0.5);
    }
    public void increasePower() {
        intakePower = Math.min(1.0, intakePower + POWER_STEP);
    }

    public void decreasePower (){
        intakePower = Math.max(0.0, intakePower - POWER_STEP);
    }
    public double getIntakeMotorPower() {
        return intakePower;
    }
    public double getIntakeRollerPower(){return robot.leftSideRoller.getPower();}

    public void stop () {
        robot.intakeMotor.setPower(0);
        robot.rightSideRoller.setPower(0);
        robot.leftSideRoller.setPower(0);
    }
}
