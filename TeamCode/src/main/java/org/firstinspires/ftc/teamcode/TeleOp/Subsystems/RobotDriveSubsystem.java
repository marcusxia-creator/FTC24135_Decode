package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class RobotDriveSubsystem extends SubsystemBase {
    private RobotHardware robot;
    public RobotDriveSubsystem (RobotHardware robot) {
        this.robot = robot;
    }

    public void mecanumDrive (double forward, double strafe, double turn){
        double FLPower = forward + strafe + turn;
        double FRPower = forward - strafe - turn;
        double BLPower = forward - strafe + turn;
        double BRPower = forward + strafe - turn;

        robot.frontLeftMotor.setPower(FLPower);
        robot.frontRightMotor.setPower(FRPower);
        robot.backLeftMotor.setPower(BLPower);
        robot.backRightMotor.setPower(BRPower);
    }
}
