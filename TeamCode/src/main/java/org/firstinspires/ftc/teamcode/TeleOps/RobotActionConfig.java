package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
public class RobotActionConfig {
    public static double powerFactor = 0.9;
    public static double accel_Slowness                 = 0.25;
    public static double decel_Slowness                 = 0.5;

    //public static PIDController = new PIDController(0,0,0);

    public static double autoHeadingCoeff               = 0.0001;
}