package org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.IWMath.floorMod;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions.angle;
import static java.lang.Math.PI;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit;

public class PositiveAngle {
    double value;

    /// Creates a positive angle object, which stores a value in SI base units and its dimension<br
    /// Angles are modulod around a full rotation, and always fall between 0° and 360°, or 0 and 2\pi radians
    public PositiveAngle(double value, Unit unit) {
        if(unit.getDimensions()==angle){
            this.value=unit.convertToSI(value);
            normalize();
        }else{
            throw new DimMismatch(unit.getDimensions(),"angle");
        }
    }

    public PositiveAngle(Scalar scalar){
        if(scalar.getDimensions()==angle){
            this.value=scalar.getValueSI();
            normalize();
        }else{
            throw new DimMismatch(scalar.getDimensions(),"angle");
        }
    }

    public PositiveAngle(NormalizedAngle normalizedAngle){
        this.value=normalizedAngle.getValueSI();
        normalize();
    }

    void normalize(){
        value=floorMod(value,2*PI);
    }

    public Scalar toScalar(){
        return new Scalar(value, angle.SIBaseUnit());
    }

    public NormalizedAngle toNormalizedAngle(){
        return new NormalizedAngle(this);
    }

    public double getValue(Unit unit) {
        return toScalar().getValue(unit);
    }

    ///@return angle in radians
    public double getValueSI() {
        return toScalar().getValueSI();
    }

    public PositiveAngle add(PositiveAngle angle){
        return new PositiveAngle(value+angle.value, Dimensions.angle.SIBaseUnit());
    }

    public PositiveAngle sub(NormalizedAngle angle){
        return new PositiveAngle(value-angle.value, Dimensions.angle.SIBaseUnit());
    }

    public PositiveAngle multiply(double factor){
        return new PositiveAngle(value*factor, angle.SIBaseUnit());
    }

    public PositiveAngle div(double factor){
        return new PositiveAngle(value/factor, angle.SIBaseUnit());
    }

    public Scalar multiply(Scalar scalar){
        return new Scalar(value*scalar.getValueSI(), angle.multiply(scalar.getDimensions()).SIBaseUnit());
    }

    public Scalar div(Scalar scalar){
        return new Scalar(value/scalar.getValueSI(), angle.div(scalar.getDimensions()).SIBaseUnit());
    }

    public Boolean greaterThan(PositiveAngle angle){
        return value>angle.value;
    }

    public Boolean greaterThanOrEqual(PositiveAngle angle){
        return value>=angle.value;
    }

    public Boolean lessThan(PositiveAngle angle){
        return value<angle.value;
    }

    public Boolean lessThanOrEqual(PositiveAngle angle){
        return value<=angle.value;
    }
}
