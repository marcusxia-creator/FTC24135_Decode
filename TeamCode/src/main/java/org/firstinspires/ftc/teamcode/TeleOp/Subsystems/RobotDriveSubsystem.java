package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class RobotDriveSubsystem extends SubsystemBase {
    double lastForward;
    double lastStrafe;
    double lastTurn;
    private RobotHardware robot;
    public RobotDriveSubsystem (RobotHardware robot) {
        this.robot = robot;
    }

    public void mecanumDrive (double forward, double strafe, double turn){
        double larp=0.2;
        forward = Range.clip(forward, lastForward-larp, lastForward+larp);
        strafe = Range.clip(strafe, lastStrafe-larp, lastStrafe+larp);
        turn = Range.clip(turn, lastTurn-larp, lastTurn+larp);

        double FLPower = forward + strafe + turn;
        double FRPower = forward - strafe - turn;
        double BLPower = forward - strafe + turn;
        double BRPower = forward + strafe - turn;

        robot.frontLeftMotor.setPower(FLPower);
        robot.frontRightMotor.setPower(FRPower);
        robot.backLeftMotor.setPower(BLPower);
        robot.backRightMotor.setPower(BRPower);

        lastForward = forward;
        lastStrafe = strafe;
        lastTurn = turn;
    }
}
