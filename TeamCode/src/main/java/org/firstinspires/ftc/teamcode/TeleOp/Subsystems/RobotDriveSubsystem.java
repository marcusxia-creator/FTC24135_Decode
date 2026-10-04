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

        lastForward = lastForward + lerp*(forward - lastForward);
        lastStrafe = lastStrafe + lerp * (strafe - lastStrafe);
        lastTurn = lastTurn + lerp * (turn - lastTurn);

        double FLPower = lastForward + lastStrafe + lastTurn;
        double FRPower = lastForward - lastStrafe - lastTurn;
        double BLPower = lastForward - lastStrafe + lastTurn;
        double BRPower = lastForward + lastStrafe - lastTurn;

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
