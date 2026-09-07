package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

public class LimelightSubsystem extends SubsystemBase {
    private RobotHardware robot;

    private double tx;
    private double ty;

    public LimelightSubsystem(RobotHardware robot) {
        this.robot = robot;
        this.robot.limelight.start();
        this.robot.limelight.pipelineSwitch(4);
    }

    public void run() {
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
        }

        else {
            tx = 0;
            ty = 0;
        }
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

}
