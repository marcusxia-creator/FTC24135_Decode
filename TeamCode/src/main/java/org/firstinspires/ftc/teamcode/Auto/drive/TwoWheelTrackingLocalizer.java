package org.firstinspires.ftc.teamcode.Auto.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;

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
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 1.0; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -1.125; // X is the up and down direction
    public static double PARALLEL_Y = -5.875; // Y is the strafe direction

    public static double PERPENDICULAR_X = -6.5;
    public static double PERPENDICULAR_Y = -0.5;

    public static double mmToinch = 0.0393701;
    Pose2D pinpointPos;
    Pose2D pinpointVel;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private GoBildaPinpointDriver pinpoint;

    public TwoWheelTrackingLocalizer(GoBildaPinpointDriver pinpoint) {
        super(Arrays.asList(
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));
        this.pinpoint = pinpoint;
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpointVel = pinpoint.getVelocity();
        pinpointPos = pinpoint.getPosition();

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
        return pinpointVel.getHeading(AngleUnit.RADIANS);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(pinpoint.getEncoderX()),
                encoderTicksToInches((pinpoint.getEncoderY())*-1)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                pinpointVel.getX(DistanceUnit.INCH),
                pinpointVel.getY(DistanceUnit.INCH)*-1
        );
    }

    @Override
    public Pose2d getPoseVelocity() {
        // Option A: Let the super class handle it if it does forward kinematics
        // return super.getPoseVelocity();

        // Option B: Return Pinpoint's velocity directly (bypassing RR's wheel-based derivative)
        Pose2D velocity = pinpoint.getVelocity();
        double xVel  = velocity.getX(DistanceUnit.INCH);
        double yVel  = velocity.getY(DistanceUnit.INCH);
        double angVel = Math.toRadians(velocity.getHeading(AngleUnit.DEGREES));

        return new Pose2d(xVel, yVel, angVel);
    }

    //updates
    @Override
    public void update() {
        super.update();     // let Road Runner do its usual steps
        pinpoint.update();  // fetch fresh data from the Pinpoint
    }
}
