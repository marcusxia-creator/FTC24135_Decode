package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.IWMath.cos;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.IWMath.sin;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.dimensionless;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit;

public class Vector {
    DimlessVector vector;
    Dimensions dimensions;

    /// Creates new vector from scalar x and y components
    /// @throws RuntimeException unitError if scalars have different dimensions
    public Vector(Scalar x, Scalar y){
        if(!x.getDimensions().equals(y.getDimensions())){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot create vector from scalars of different dimensions %s, and SI base units %s and %s and SI base units %s",x.getDimensions().toString(), x.getDimensions().SIBaseUnitStr(), y.getDimensions().toString(), y.getDimensions().SIBaseUnitStr()));}
        vector=new DimlessVector(x.getValueSI(), y.getValueSI());
        this.dimensions=x.getDimensions();
    }

    /// Creates new vector from dimensionless x and y components and a unit
    public Vector(double x, double y, Unit unit){
        this(new Scalar(x,unit),new Scalar(y,unit));
    }

    /// Creates new vector from a dimensionless vector and a unit
    public Vector(DimlessVector dimlessVector, Unit unit){
        this(dimlessVector.getX(),dimlessVector.getY(),unit);
    }

    public Dimensions getDimensions(){
        return dimensions;
    }

    public DimlessVector getDimlessVector(){
        return vector;
    }

    /// @return magnitude as a scalar
    public Scalar mag(){
        return new Scalar(vector.mag(), dimensions.SIBaseUnit());
    }

    /// @return the unit vector with the same direction as the vector, but with magnitude of 1
    /// @throws RuntimeException "mathError" if the vector has a magnitude of 0
    public DimlessVector unitVector(){
        return vector.unitVector();
    }

    ///Returns a normalized angle representing the direction of the vector
    /// note: All angles are measured from the positive y-axis towards the positive x-axis (clockwise)
    public NormalizedAngle direction(){
        return vector.direction();
    }

    /// @throws RuntimeException unitError if vectors have different dimensions
    public Vector add(Vector vector){
        if(!dimensions.equals(vector.getDimensions())){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot add vectors with unit of dimensions %s and SI base units %s and unit of dimensions %s and SI base units %s",vector.getDimensions().toString(), vector.getDimensions().SIBaseUnitStr(), dimensions.toString(), dimensions.SIBaseUnitStr()));}
        return new Vector(this.vector.add(vector.getDimlessVector()),dimensions.SIBaseUnit());
    }

    /// @throws RuntimeException unitError if vectors have different dimensions
    public Vector sub(Vector vector){
        if(!dimensions.equals(vector.getDimensions())){throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot subtract vectors with unit of dimensions %s and SI base units %s and unit of dimensions %s and SI base units %s",vector.getDimensions().toString(), vector.getDimensions().SIBaseUnitStr(), dimensions.toString(), dimensions.SIBaseUnitStr()));}
        return new Vector(this.vector.sub(vector.getDimlessVector()),dimensions.SIBaseUnit());
    }

    public Vector multiply(Scalar scalar){
        return new Vector(this.vector.multi(scalar.getValueSI()),dimensions.multiply(scalar.getDimensions()).SIBaseUnit());
    }

    public Vector div(Scalar scalar){
        return new Vector(this.vector.div(scalar.getValueSI()),dimensions.div(scalar.getDimensions()).SIBaseUnit());
    }

    public Vector multiply(double factor){
        return multiply(new Scalar(factor, dimensionless.SIBaseUnit()));
    }

    public Vector div(double factor){
        return div(new Scalar(factor, dimensionless.SIBaseUnit()));
    }

    /// Rotates the vector by normalized angle {@code angle} clockwise
    /// @param angle the angle to rotate by
    /// @return the rotated vector
    public Vector rotateBy(NormalizedAngle angle){
        return new Vector(vector.rotateBy(angle), dimensions.SIBaseUnit());
    }
}