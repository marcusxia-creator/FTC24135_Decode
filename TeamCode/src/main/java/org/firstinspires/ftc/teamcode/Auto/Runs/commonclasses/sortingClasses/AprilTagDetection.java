package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import java.util.List;

public class AprilTagDetection {
    private final RobotHardware robot;

    public int tagID = -1;

    public AprilTagDetection (RobotHardware robot){
        this.robot = robot;
    }

    public void limelightStart () {
        robot.limelight.pipelineSwitch(2);
        robot.limelight.start();
    }

    public void limelightDetect () {
        LLResult llResult = robot.limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            tagID = fiducial.getFiducialId(); // The ID number of the fiducial
        }
    }

    public int findGreenSlotStandard() {
        if (tagID == 21) {
            return 0;
        } else if (tagID == 22) {
            return 1;
        } else if (tagID == 23) {
            return 2;
        } else {
            return -1;
        }
    }

    public int findGreenSlotCloseRed () {
        if (tagID == 21) {
            ///Actual Tag ID = 22
            return 1;
        } else if (tagID == 22) {
            ///Actual Tag ID = 23
            return 2;
        } else if (tagID == 23) {
            ///Actual Tag ID = 21
            return 0;
        } else {
            return -1;
        }
    }

    public int findGreenSlotCloseBlue () {
        if (tagID == 21) {
            ///Actual Tag ID = 23
            return 2;
        } else if (tagID == 22) {
            ///Actual Tag ID = 21
            return 0;
        } else if (tagID == 23) {
            ///Actual Tag ID = 22
            return 1;
        } else {
            return -1;
        }
    }
}