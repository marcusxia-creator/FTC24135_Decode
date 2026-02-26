package org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.sortingClasses.AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
public class AutoLimelightDetection {
    private final RobotHardware robot;
    private final AprilTagDetection aprilTagDetection;

    public static int targetGreenSlot;
    public static int tagID;

    public AutoLimelightDetection (RobotHardware robot, AprilTagDetection aprilTagDetection) {
        this.robot = robot;
        this.aprilTagDetection = aprilTagDetection;
    }

    public static class AutoLimelightRun implements Action {
        public enum DETECTION_STATE {
            DETECTION_INIT,
            DETECTION_RUN,
            DETECTION_DETECT,
            DETECTION_STOP
        }

        private final RobotHardware robot;
        private final AprilTagDetection aprilTagDetection;
        public DETECTION_STATE currentState;

        private final ElapsedTime detectionTime = new ElapsedTime();
        private final ElapsedTime stateTimer = new ElapsedTime();

        public final double runTime;

        public AutoLimelightRun(RobotHardware robot, AprilTagDetection aprilTagDetection, double RunTime) {
            this.robot = robot;
            this.aprilTagDetection = aprilTagDetection;
            this.runTime = RunTime;
        }

        public void FSMDetectionRun() {
            switch (currentState) {
                case DETECTION_INIT:
                    aprilTagDetection.limelightStart();
                    detectionTime.reset();
                    stateTimer.reset();
                    currentState = DETECTION_STATE.DETECTION_RUN;
                    break;
                case DETECTION_RUN:
                    if (detectionTime.seconds() > runTime) {
                        stateTimer.reset();
                        currentState = DETECTION_STATE.DETECTION_STOP;
                        break;
                    } else {
                        stateTimer.reset();
                        currentState = DETECTION_STATE.DETECTION_RUN;
                        break;
                    }
                case DETECTION_DETECT:
                    aprilTagDetection.limelightDetect();
                    targetGreenSlot = aprilTagDetection.findGreenSlot();
                    tagID = aprilTagDetection.tagID;
                    stateTimer.reset();
                    currentState = DETECTION_STATE.DETECTION_RUN;
                    break;
                case DETECTION_STOP:
                    break;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            FSMDetectionRun();
            telemetryPacket.put("FSM Detection State", currentState);
            telemetryPacket.put("Tag ID", tagID);
            telemetryPacket.put("Target Green Slot", targetGreenSlot);
            return currentState != DETECTION_STATE.DETECTION_STOP;
        }
    }

    public Action limelightDetect(RobotHardware robotHardware, AprilTagDetection aprilTagDetection, double RunTime) {
        return new AutoLimelightRun(robotHardware, aprilTagDetection, RunTime);
    }

    public int getTargetGreenSlot () {
        return targetGreenSlot;
    }

    public int getTagID () {
        return tagID;
    }
}