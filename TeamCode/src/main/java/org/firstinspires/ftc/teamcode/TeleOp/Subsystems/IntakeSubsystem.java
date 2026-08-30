package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;
public class IntakeSubsystem extends SubsystemBase{
    private RobotHardware robot;
    private double intakePower = 0.7;
    private final double POWER_STEP = 0.1;
    public IntakeSubsystem (RobotHardware robot) {
        this.robot = robot;
    }
    public void runRollers (){
        robot.intakeMotor.setPower(intakePower);
    }
    public void increasePower() {
        intakePower = Math.min(1.0, intakePower + POWER_STEP);
    }

    public void decreasePower (){
        intakePower = Math.max(0.0, intakePower - POWER_STEP);
    }
    public double getIntakePower() {
        return intakePower;
    }

    public void stop () {
        robot.intakeMotor.setPower(0);
    }
}
