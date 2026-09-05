package org.firstinspires.ftc.teamcode.TeleOp.UniversalTools;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class RobotActionConfig {
    ///Lift Subsystem
    public static double Ticks_Per_Moto_Revolution = 145.1;
    public static double Pulley_Ratio = 2.0 / 3.0;
    public static double Actual_Ticks_Per_Revolution = Ticks_Per_Moto_Revolution * Pulley_Ratio; //96.7
    public static double Spool_Circumference_MM = 32 * Math.PI; //100.5MM
    public static final double Ticks_Per_MM = Actual_Ticks_Per_Revolution / Spool_Circumference_MM; //0.96

}
