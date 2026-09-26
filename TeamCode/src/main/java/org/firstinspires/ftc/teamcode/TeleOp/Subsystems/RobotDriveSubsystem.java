package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

public class RobotDriveSubsystem extends SubsystemBase {
    double lastForward;
    double lastStrafe;
    double lastTurn;
    private RobotHardware robot;
    public RobotDriveSubsystem (RobotHardware robot) {
        this.robot = robot;
    }
    public void mecanumDrive (double forward, double strafe, double turn){
        double lerp =0.15;
        forward = Range.clip(forward, lastForward- lerp, lastForward+ lerp);
        strafe = Range.clip(strafe, lastStrafe- lerp, lastStrafe+ lerp);
        turn = Range.clip(turn, lastTurn- lerp, lastTurn+ lerp);

        lastForward = forward;
        lastStrafe = strafe;
        lastTurn = turn;

        double FLPower = forward + strafe + turn;
        double FRPower = forward - strafe - turn;
        double BLPower = forward - strafe + turn;
        double BRPower = forward + strafe - turn;

        double max = Math.max(Math.abs(FLPower), Math.max(Math.abs(FRPower),
                Math.max(Math.abs(BLPower), Math.abs(BRPower))));
        if (max > 1.0) {
            FLPower /= max;
            FRPower /= max;
            BLPower /= max;
            BRPower /= max;
        }

        robot.frontLeftMotor.setPower(FLPower);
        robot.frontRightMotor.setPower(FRPower);
        robot.backLeftMotor.setPower(BLPower);
        robot.backRightMotor.setPower(BRPower);
    }
}
