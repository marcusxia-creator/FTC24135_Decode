package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

public class MotifDetector {

    private RobotHardware robot;
    private Turret turret;

    public MotifDetector (RobotHardware robot, Turret turret) {
        this.robot = robot;
        this.turret = turret;
    }

    public void initLimelight() {
        robot.limelight3A.pipelineSwitch(2);
    }

    public void start() {
        robot.limelight3A.start();
    }

    public int update() {
        robot.limelight3A.updateRobotOrientation(turret.getTurretMotorAngle());
        LLResult result = robot.limelight3A.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            if (id == 21) return 1;
            if (id == 22) return 2;
            if (id == 23) return 3;
        }

        return 1;
    }
}
