package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.*;
import static java.lang.Math.PI;

public class Unit {
    double factor;
    Dimensions dimensions;

    /// Creates a unit storage object,which stores a factor relative to base SI units, and it's dimensions<br>
    /// @param factor The unit's factor relative to its corresponding unit in SI base units with the same dimensions, as a combination of Meters, Radians, and Seconds. <br>
    /// **Example**: The unit km/h should have a factor {@code 0.277778}, of units (m/s)(km/h)
    /// @param dimensions The dimensions of the unit represented as a Dimensions object. These dimensions are used to restrict unit operations and conversions.<br>
    /// **Example** The unit km/h has dimensions space per time, represented by dimension object {@code new dimensions(1.0.-1)}
    public Unit(double factor, Dimensions dimensions){
        this.factor=factor;
        this.dimensions=dimensions;
    }

    public Unit pow(double exponent){
        return new Unit(Math.pow(factor,exponent),dimensions.pow(exponent));
    }

    public Unit multiply(Unit unit){
        return new Unit(factor*unit.factor,dimensions.multiply(unit.dimensions));
    }

    public Unit div(Unit unit){
        return multiply(unit.pow(-1));
    }

    /// @return conversion factor to base SI units <br>
    /// **Example**: {@code mm.FactorToM();} returns {@code 0.001} (m/mm)
    /// Multiply value in unit by this factor to get value in SI base units
    public double getFactorToSI(){
        return factor;
    }

    /// @return conversion factor to unit from base SI units <br>
    /// **Example**: {@code mm.FactorToM();} returns {@code 1000} (mm/m)
    /// Multiply value in SI units by this factor to get value in the desired unit
    public double getFactorFromSI(){
        return 1/factor;
    }

    /// Returns conversion factor to another unit, as long as dimensions are the same <br>
    /// @return conversion factor as a double
    /// **Example**: {@code mm.FactorToM();} returns {@code 0.001} (m/mm)
    /// Multiply original value in original unit by this factor to get value in new units
    /// @throws RuntimeException unitError if dimensions do not match
    public double getFactorToUnit(Unit unit){
        if(dimensions.equals(unit.dimensions)){
            return getFactorToSI()/unit.getFactorToSI();
        }
        else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot convert unit of dimensions %s and SI base units %s to new dimensions %s and SI base units %s",dimensions.toString(), dimensions.SIBaseUnitStr(), unit.dimensions.toString(), unit.dimensions.SIBaseUnitStr()));
        }
    }

    /// Converts parameter {@code value} from self unit to SI base units
    /// @return Converted value
    public double convertToSI(double value){
        return value*getFactorToSI();
    }

    /// Converts parameter {@code value} from current unit to parameter {@code unitTo}
    /// @param unitTo The unit to convert to
    /// @return Converted value
    /// @throws RuntimeException unitError if dimensions do not match
    public double convertToUnit(double value, Unit unitTo){
        return value*getFactorToUnit(unitTo);
    }

    /// Converts parameter {@code value} from base SI units to self unit
    /// @return Converted value
    public double convertFromSI(double value){
        return value*getFactorFromSI();
    }

    public Dimensions getDimensions(){
        return dimensions;
    }

    //Premade Units
    public static Unit none =new Unit(1,        dimensionless);
    //Lengths
    public static Unit mm   =new Unit(0.001,    length);
    public static Unit cm   =new Unit(0.01,     length);
    public static Unit in   =new Unit(0.0254,   length);
    public static Unit feet =new Unit(0.3048,   length);
    public static Unit yard =new Unit(0.9144,   length);
    public static Unit m    =new Unit(1,        length);
    public static Unit field=new Unit(3.66,     length);
    public static Unit km   =new Unit(1000,     length);

    //Angles
    public static Unit arcsec   =new Unit(PI/648000,angle);
    public static Unit arcmin   =new Unit(PI/10800, angle);
    public static Unit deg      =new Unit(PI/180,   angle);
    public static Unit rad      =new Unit(1,        angle);
    public static Unit rev      =new Unit(PI*2,     angle);

    //Times
    public static Unit ns   =new Unit(1e-9, time);
    public static Unit ms   =new Unit(0.001,time);
    public static Unit s    =new Unit(1,    time);
    public static Unit min  =new Unit(60,    time);
    public static Unit hour =new Unit(3600,  time);

    //Others
    public static Unit metersPerSecond=m.div(s);
    public static Unit kmPerHour=km.div(hour);
    public static Unit degreesPerSecond=deg.div(s);
    public static Unit radiansPerSecond=rad.div(s);
    public static Unit rpm=rev.div(s);
    public static Unit metersPerSecondSquared=m.div(s.pow(2));
    public static Unit degreesPerSecondSquared=deg.div(s.pow(2));
    public static Unit radiansPerSecondSquared=rad.div(s.pow(2));
}