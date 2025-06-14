package org.firstinspires.ftc.teamcode.Auto.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class PinpointTwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.629921; // in
    public static double GEAR_RATIO = 1.0; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -6; ///-0.875; // X is the up and down direction
    public static double PARALLEL_Y = 3.75; ///-5.875; // Y is the strafe direction

    public static double PERPENDICULAR_X = -6; ///-5.1672;
    public static double PERPENDICULAR_Y = -4; ///-1;

    public static double x_offset = -149.225; // unit in mm    or change to -5.875=149.225
    public static double y_offset = -131.2468; // unit in mm     or change to -5.1672=131.2468

    Pose2D pinpointPos;
    Pose2D pinpointVel;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private GoBildaPinpointDriver pinpoint;

    public PinpointTwoWheelTrackingLocalizer(GoBildaPinpointDriver pinpoint) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));
        this.pinpoint = pinpoint;
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        this.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        this.pinpoint.setOffsets(x_offset,y_offset);
        this.pinpoint.resetPosAndIMU();
    }
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return pinpoint.getHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return pinpoint.getHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches((pinpoint.getEncoderX()*-1)),
                encoderTicksToInches((pinpoint.getEncoderY())*-1)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        pinpoint.update();
        double heading =  pinpoint.getHeading();
        double cosA = Math.cos(heading);
        double sinA = Math.sin(heading);
        // Extract the world velocity components
        double worldVx = pinpoint.getVelX()/25.4;  //convert to inch/s
        double worldVy = pinpoint.getVelY()/25.4;  //convert to inch/s

        // Apply the rotation (note the use of heading rather than -heading due to trigonometric identities)
        double robotVx = worldVx * cosA + worldVy * sinA;
        double robotVy = -worldVx * sinA + worldVy * cosA;
        return Arrays.asList(
                robotVx,
                robotVy
        );
    }
}


