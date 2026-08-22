package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem;

/**
 * Field-frame velocity of the robot at one instant, matching the same
 * X/Y/heading convention DriveMotionController already uses for Pose2D
 * position.
 *
 * This is a plain measurement: it carries no rotation into the robot
 * frame. DriveMotionController performs field-to-robot rotation only
 * once, at the very end of its control loop.
 *
 * Units:
 * velocity: millimetres per second
 * heading velocity: radians per second
 */
public final class RobotVelocity {

    private final double vxMMPerSecond;
    private final double vyMMPerSecond;
    private final double headingVelocityRadiansPerSecond;

    public RobotVelocity(
            double vxMMPerSecond,
            double vyMMPerSecond,
            double headingVelocityRadiansPerSecond
    ) {
        this.vxMMPerSecond = vxMMPerSecond;
        this.vyMMPerSecond = vyMMPerSecond;
        this.headingVelocityRadiansPerSecond = headingVelocityRadiansPerSecond;
    }

    public double getVxMMPerSecond() {
        return vxMMPerSecond;
    }

    public double getVyMMPerSecond() {
        return vyMMPerSecond;
    }

    public double getHeadingVelocityRadiansPerSecond() {
        return headingVelocityRadiansPerSecond;
    }
}
