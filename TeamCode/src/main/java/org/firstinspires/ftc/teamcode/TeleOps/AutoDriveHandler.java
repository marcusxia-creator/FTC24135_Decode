package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Auto.PointToDrive;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDriveCancelable;

public class AutoDriveHandler {

    private SampleMecanumDriveCancelable drive;
    private Pose2d poseEstimate;
    private int n;

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

        // Create the target vector and heading.
        Vector2d targetAVector = new Vector2d(target_X, PointToDrive.highbar_y_coordinate);
        double targetAHeading = Math.toRadians(-90);

        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                .splineTo(targetAVector, targetAHeading)
                .build();
        // Validate position ranges before following trajectory.
        if (((X > 0) || (X < 60)) && ((Y > 12) || (Y < 72))) {
            drive.followTrajectoryAsync(traj1);
            n++; // increment n for next use
            return true; // indicate that auto-drive was started
        }
        return false;
    }

    /**
     * Executes the auto-drive action for the A button.
     * @return true if the auto drive was initiated.
     */
    public boolean handleButtonA() {
        double X = Math.abs(poseEstimate.getX());
        double Y = Math.abs(poseEstimate.getY());
        Vector2d targetBVector = new Vector2d(PointToDrive.specimen_pickup_x_coordinate,
                PointToDrive.specimen_pickup_y_coordinate);
        double targetAHeading = Math.toRadians(-45);

        Trajectory traj2 = drive.trajectoryBuilder(poseEstimate)
                .splineTo(targetBVector, targetAHeading)
                .build();
        if ((((13 - X) >= 0) || (X > 13)) && ((Y > 12) || (Y < 72))) {
            drive.followTrajectoryAsync(traj2);
            return true;
        }
        return false;
    }

    // Optionally, you might provide setter methods to update poseEstimate
    public void updatePose(Pose2d newPose) {
        this.poseEstimate = newPose;
    }
}

