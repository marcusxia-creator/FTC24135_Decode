package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.IWMath.floorMod;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.angle;
import static java.lang.Math.PI;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit;

public class NormalizedAngle{
    double value;

    /// Creates a normalized angle object, which stores a value in SI base units and its dimension<br
    /// Angles are normalized about 0 radians or degrees, and always fall between -180° and 180°, or -\pi and \pi radians
    public NormalizedAngle(double value, Unit unit) {
        if(unit.getDimensions()==angle){
            this.value=unit.convertToSI(value);
            normalize();
        }else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot store unit of dimensions %s and SI base units %s as a normalized object",unit.getDimensions().toString(), unit.getDimensions().SIBaseUnitStr()));
        }
    }

    public NormalizedAngle(Scalar scalar){
        if(scalar.getDimensions()==angle){
            this.value=scalar.getValueSI();
            normalize();
        }else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot store scalar of dimensions %s and SI base units %s as a normalized object",scalar.getDimensions().toString(), scalar.getDimensions().SIBaseUnitStr()));
        }
    }

    void normalize(){
        value=floorMod(value+PI,2*PI)-PI;
    }

    public Scalar toScalar(){
        return new Scalar(value, angle.SIBaseUnit());
    }

    public double getValue(Unit unit) {
        return toScalar().getValue(unit);
    }

    ///@return angle in radians
    public double getValueSI() {
        return toScalar().getValueSI();
    }

    public NormalizedAngle add(NormalizedAngle angle){
        return new NormalizedAngle(value+angle.value, Dimensions.angle.SIBaseUnit());
    }

    public NormalizedAngle sub(NormalizedAngle angle){
        return new NormalizedAngle(value-angle.value, Dimensions.angle.SIBaseUnit());
    }

    public NormalizedAngle multiply(double factor){
        return new NormalizedAngle(value*factor, angle.SIBaseUnit());
    }

    public NormalizedAngle div(double factor){
        return new NormalizedAngle(value/factor, angle.SIBaseUnit());
    }

    public Scalar multiply(Scalar scalar){
        return new Scalar(value*scalar.getValueSI(), angle.multiply(scalar.getDimensions()).SIBaseUnit());
    }

    public Scalar div(Scalar scalar){
        return new Scalar(value/scalar.getValueSI(), angle.div(scalar.getDimensions()).SIBaseUnit());
    }
}
