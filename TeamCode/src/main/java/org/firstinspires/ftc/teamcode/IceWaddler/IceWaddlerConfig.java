package org.firstinspires.ftc.teamcode.IceWaddler;

import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class IceWaddlerConfig {
    //Odometry Settings
    public static double odoXOffset                                             = -149.225;
    public static double odoYOffset                                             = -165.1;
    public static GoBildaPinpointDriver.GoBildaOdometryPods odoEncoderResolution= GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    public static GoBildaPinpointDriver.EncoderDirection xEncoderDirection      = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yEncoderDirection      = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static PIDCoefficients vController = new PIDCoefficients(0.6, 0, 0);
    public static PIDCoefficients vRotController = new PIDCoefficients(0.08, 0, 0);

    public static PIDCoefficients pController = new PIDCoefficients(10, 0, 0);
    public static PIDCoefficients pRotController = new PIDCoefficients(10, 0, 0);

    //Positional control parameters
    public static double maxSpeed = 2;
    public static double minSpeed = 0.1;
    public static double maxDecel = 1.3;
    public static double tolerance = 0.005;
}