package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math;

import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;

public class IWMath{
    public static double floorMod(double value, double modBy){
        return value-modBy*floor(value/modBy);
    }
    public static double sin(NormalizedAngle normalizedAngle){
        return Math.sin(normalizedAngle.getValueSI());
    }

    public static double cos(NormalizedAngle normalizedAngle){
        return Math.cos(normalizedAngle.getValueSI());
    }

    public static double tan(NormalizedAngle normalizedAngle){
        return Math.tan(normalizedAngle.getValueSI());
    }

    public static NormalizedAngle arcSin(double value){
        return new NormalizedAngle(asin(value),rad);
    }

    public static NormalizedAngle arcCos(double value){
        return new NormalizedAngle(acos(value),rad);
    }

    ///Information is lost in division, only returns angle between -90° and 90°. It's better to use arcTan2
    public static NormalizedAngle arcTan(double value){
        return new NormalizedAngle(atan(value),rad);
    }

    /// Unlike other code, outputs from this function are measured from the x-axis counterclockwise, to stay consistent other trigonometric functions.
    /// Mitigate this discrepancy by adjusting parameter order and signs<br>
    /// Output examples:<br>
    /// {@code (1,0)}   outputs {@code 0.0°}<br>
    /// {@code (1,1)}   outputs {@code 45.0°}<br>
    /// {@code (0,1)}   outputs {@code 90.0°}<br>
    /// {@code (-1,0)}  outputs {@code 180.0°}<br>
    /// {@code (0,-1)}  outputs {@code -90.0°}<br>
    /// {@code (-1,-1)} outputs {@code -45.0°}<br>
    public static NormalizedAngle arcTan2(double x, double y){
        return new NormalizedAngle(atan2(y,x),rad);
    }
}
