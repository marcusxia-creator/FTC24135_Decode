package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Auto.PointToDrive;
import org.firstinspires.ftc.teamcode.Auto.RightSideAuto_4Specimen;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;

public class AutoDriveHandler {

    private SampleMecanumDriveCancelable drive;
    private Pose2d poseEstimate;
    private int n;
    private RightSideAuto_4Specimen rightAuto;

    public AutoDriveHandler(SampleMecanumDriveCancelable drive, Pose2d poseEstimate, int initialN) {
        this.drive = drive;
        this.poseEstimate = poseEstimate;
        this.n = initialN;
    }

    /**
     * Executes the auto-drive action for the Y button.
     * @return true if the auto drive was initiated.
     */
    public boolean handleButtonY() {
        double X = Math.abs(poseEstimate.getX());
        double Y = Math.abs(poseEstimate.getY());
        double offset = ((n - 1) % 10) + 1.5;
        double target_X = PointToDrive.highbar_x_coordinate_left + offset;

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(poseEstimate)
                .lineToLinearHeading(new Pose2d(target_X, PointToDrive.highbar_y_coordinate, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{rightAuto.extendDepositSysScoring();})
                .build();
        // Validate position ranges before following trajectory.
        if (((X > 0) || (X < 60)) && ((Y > 12) || (Y < 72))) {
            drive.followTrajectorySequence(traj1);
            n++; // increment n for next use
            return true; // indicate that auto-drive was started
        }
        return false;
    }

    /**
     * Executes the auto-drive action for the A button.
     * Drive to Specimen Pick up position
     * @return true if the auto drive was initiated.
     */
    public boolean handleButtonA() {
        double X = Math.abs(poseEstimate.getX());
        double Y = Math.abs(poseEstimate.getY());
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(poseEstimate)
                .lineToLinearHeading(new Pose2d(PointToDrive.specimen_pickup_x_coordinate, PointToDrive.specimen_pickup_y_coordinate, Math.toRadians(-45)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{rightAuto.depositSysRetract();})
                .UNSTABLE_addTemporalMarkerOffset(0,()->{rightAuto.intakeSpecimenPick();})
                .build();
        if ((((13 - X) >= 0) || (X > 13)) && ((Y > 12) || (Y < 72))) {
            drive.followTrajectorySequence(traj2);
            return true;
        }
        return false;
    }

    // Optionally, you might provide setter methods to update poseEstimate
    public void updatePose(Pose2d newPose) {
        this.poseEstimate = newPose;
    }
}

