package org.firstinspires.ftc.teamcode.Auto.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class PinpointCustomLocalizer implements Localizer {

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 1.0; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -1; ///-0.875; // X is the up and down direction
    public static double PARALLEL_Y = -6.5; ///-5.875; // Y is the strafe direction

    public static double PERPENDICULAR_X = -6; ///-5.1672;
    public static double PERPENDICULAR_Y = -0.875; ///-1;

    public static double x_offset = -149.225; // unit in mm
    public static double y_offset = -165.1; // unit in mm

    private Pose2d txWorldPinpoint;                                     // Global FTC field coordinate
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0); //  this is hte robot coordinate

    private GoBildaPinpointDriver pinpoint;

    public PinpointCustomLocalizer(GoBildaPinpointDriver pinpoint, Pose2d initialPose){
        this.pinpoint = pinpoint;
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //
        this.pinpoint.setOffsets(x_offset,y_offset);
        this.pinpoint.resetPosAndIMU();
        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        double cosTheta = Math.cos(txPinpointRobot.getHeading());
        double sinTheta = Math.sin(txPinpointRobot.getHeading());
        double invX = - (pose2d.getX() * cosTheta + pose2d.getY() * sinTheta);
        double invY =   pose2d.getX() * sinTheta - pose2d.getY() * cosTheta;
        double invHeading = -pose2d.getHeading();
        txWorldPinpoint = new Pose2d(invX, invY, invHeading);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        txWorldPinpoint = new Pose2d(pinpoint.getPosition().getX(DistanceUnit.INCH),pinpoint.getPosition().getY(DistanceUnit.INCH), pinpoint.getHeading());
       return txWorldPinpoint;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        pinpoint.update();
        if (Objects.requireNonNull(pinpoint.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(pinpoint.getPosition().getX(DistanceUnit.INCH), pinpoint.getPosition().getY(DistanceUnit.INCH), pinpoint.getHeading());
            Vector2d worldVelocity = new Vector2d(pinpoint.getVelocity().getX(DistanceUnit.INCH), pinpoint.getVelocity().getY(DistanceUnit.INCH));

            double heading =  pinpoint.getHeading();
            double cosA = Math.cos(heading);
            double sinA = Math.sin(heading);
            // Extract the world velocity components
            double worldVx = pinpoint.getVelX()/25.4;  //convert to inch/s
            double worldVy = pinpoint.getVelY()/25.4;  //convert to inch/s

            // Apply the rotation (note the use of heading rather than -heading due to trigonometric identities)
            double robotVx = worldVx * cosA + worldVy * sinA;
            double robotVy = (-worldVx) * sinA + worldVy * cosA;

            Vector2d robotVelocity= new Vector2d(robotVx, robotVy);
            return new Pose2d(robotVelocity, pinpoint.getHeadingVelocity());
        }
        return new Pose2d(new Vector2d(0, 0), 0);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    @NonNull
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(pinpoint.getEncoderX()),
                encoderTicksToInches((pinpoint.getEncoderY())*-1)
        );
    }

    @Override
    public void update() {

    }
}
