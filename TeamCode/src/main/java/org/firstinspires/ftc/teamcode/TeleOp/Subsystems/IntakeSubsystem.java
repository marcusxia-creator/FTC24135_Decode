package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;
public class IntakeSubsystem extends SubsystemBase{
    private RobotHardware robot;
    public IntakeSubsystem (RobotHardware robot) {
        this.robot = robot;
    }
    public void runRollers (double power){
        robot.intakeMotor.setPower(power);
    }

    public void stop () {
        robot.intakeMotor.setPower(0);
    }
}
