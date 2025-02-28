package org.firstinspires.ftc.teamcode.Auto.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

public class PinpointCumstomLocalizer implements Localizer {

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

    public PinpointCumstomLocalizer(GoBildaPinpointDriver pinpoint, Pose2d initialPose){
        this.pinpoint = pinpoint;
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //
        this.pinpoint.setOffsets(x_offset,y_offset);

        this.pinpoint.resetPosAndIMU();
    }
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return null;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {

    }
}
