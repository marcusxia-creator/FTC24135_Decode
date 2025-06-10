/***package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.Auto.RightSideAuto_3Specimen.hSlideWaitTimer;
import static org.firstinspires.ftc.teamcode.Auto.RightSideAuto_3Specimen.wristWaitTimer;
import static org.firstinspires.ftc.teamcode.TeleOps.BasicTeleOps_SemiAuto.initialRun;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.PointToDrive;
import org.firstinspires.ftc.teamcode.Auto.RightSideAuto_4Specimen_Original;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;

public class AutoDriveHandler {

    private SampleMecanumDriveCancelable drive;
    private Pose2d poseEstimate;
    private int n;
    private RightSideAuto_4Specimen_Original rightAuto;

    private VerticalSlide vSlides;

    private RobotHardware robot;

    private final FiniteStateMachineDeposit depositArmDrive;

    private ElapsedTime holdTimer = new ElapsedTime();

    public AutoDriveHandler(SampleMecanumDriveCancelable drive, RobotHardware robot, int initialN, FiniteStateMachineDeposit depositArmDrive) {
        this.drive = drive;
        this.robot = robot;
        this.depositArmDrive = depositArmDrive;
        vSlides = new VerticalSlide(this.robot);
        this.n = initialN;
        this.poseEstimate = new Pose2d(0, 0, 0); // Default placeholder pose
    }

    /**
     * Updates poseEstimate dynamically.

    public void updatePoseEstimate(Pose2d newPose) {
        this.poseEstimate = newPose;
    }

    /**
     * Executes the auto-drive action for the Y button.
     * @return true if the auto drive was initiated.

    public boolean handleButtonY() {
        double X = poseEstimate.getX();
        double Y = poseEstimate.getY();
        if (!((X > -10) & (X < 60) && (Y < - 12) && (Y >-72))) {
            return false;
        }
        double offset = ((n - 1) % 10) + 1.5;
        double target_X = PointToDrive.highbar_x_coordinate_left + offset;
        TrajectorySequence traj_Initial = drive.trajectorySequenceBuilder(poseEstimate)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    vSlides.slidesMove(RobotActionConfig.deposit_Slide_Highbar_Pos, RobotActionConfig.deposit_Slide_UpLiftPower);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);
                })
                .lineToLinearHeading(new Pose2d(target_X, PointToDrive.highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);
                })
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(PointToDrive.specimen_pickup_x_coordinate, PointToDrive.specimen_pickup_y_coordinate, Math.toRadians(-35)))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    vSlides.slidesMoveDown(RobotActionConfig.deposit_Slide_Down_Pos, 0.8);
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    depositArmDrive.SetDepositClawState(FiniteStateMachineDeposit.DEPOSITCLAWSTATE.OPEN);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    vSlides.Slides_Stop();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence traj_normal = drive.trajectorySequenceBuilder(poseEstimate)
                .addTemporalMarker(()->{
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
                })
                .waitSeconds(hSlideWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Transfer);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                })
                .waitSeconds(wristWaitTimer)
                .addTemporalMarker(() -> {
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                })
                .waitSeconds(0.9)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{vSlides.slidesMove(RobotActionConfig.deposit_Slide_Highbar_Pos,RobotActionConfig.deposit_Slide_UpLiftPower);})
                .UNSTABLE_addTemporalMarkerOffset(0,()->{robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Hook);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Hook);})
                .lineToLinearHeading(new Pose2d(target_X, PointToDrive.highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})
                .UNSTABLE_addTemporalMarkerOffset(0.15,()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Flat_Pos);})
                .waitSeconds(0.4)
                .lineToLinearHeading(new Pose2d(PointToDrive.specimen_pickup_x_coordinate, PointToDrive.specimen_pickup_y_coordinate, Math.toRadians(-33)))
                .UNSTABLE_addTemporalMarkerOffset(-0.75,()->{
                    vSlides.slidesMoveDown(RobotActionConfig.deposit_Slide_Down_Pos, 0.8);
                    robot.depositLeftArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositRightArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    depositArmDrive.SetDepositClawState(FiniteStateMachineDeposit.DEPOSITCLAWSTATE.OPEN);
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Wait);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{vSlides.Slides_Stop();})
                .waitSeconds(1)
                .build();
        // Validate position ranges before following trajectory.
        if (initialRun) {
            drive.followTrajectorySequenceAsync(traj_Initial);
            initialRun = false;
        } else {
            drive.followTrajectorySequenceAsync(traj_normal);
            n++;
        }
        return true;
    }

    private void intakeSpecimenPickReady(){
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension_Wait);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Wait);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
    }

    private void intakeSpecimenPick(){
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Pick);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
    }

    /** Slides subclass for vertical slides moving helper method.
    class VerticalSlide {
        RobotHardware robot;
        private VerticalSlide(RobotHardware robot){
            this.robot = robot;
        }
        private void Slides_Stop() {
            robot.liftMotorLeft.setPower(0);
            robot.liftMotorRight.setPower(0);
        }

        private void slidesMove(double targetPosition, double speed) {
            int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
            robot.liftMotorLeft.setTargetPosition(targetTick);
            robot.liftMotorRight.setTargetPosition(targetTick);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }

        private void slidesMoveDown(double targetPosition, double speed) {
            int targetTick = (int) (targetPosition * RobotActionConfig.TICKS_PER_MM_Slides);
            robot.liftMotorLeft.setTargetPosition(targetTick);
            robot.liftMotorRight.setTargetPosition(targetTick);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);

            /*while (robot.liftMotorLeft.isBusy() || robot.liftMotorRight.isBusy()) {
                if (lSisPressed()) {  // Pass actual time
                    Slides_Stop();
                    break;
                }
            }


        }

    }
    /**HELPER METHODS*/
    /**
     * Returns true only if the limit switch has been pressed continuously for at least 200ms.


    private boolean lSisPressed() {
        boolean switchState = robot.limitSwitch.getState(); // Read switch state

        if (switchState) {
            if (holdTimer.milliseconds() >= RobotActionConfig.debounceDelay) {
                return true; // Return true only if held for at least 200ms
            }
        } else {
            holdTimer.reset(); // Reset timer when switch is released
        }
        return false;
    }
} */

