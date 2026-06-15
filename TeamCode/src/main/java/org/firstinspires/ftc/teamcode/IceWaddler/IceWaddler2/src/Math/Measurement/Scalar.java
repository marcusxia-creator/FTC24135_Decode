package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit;

public class Scalar {
    /// The value of the scalar, in SI base units
    double value;

    Dimensions dimensions;

    ///Creates a Scalar object, which stores a value in SI base units and its dimensions
    public Scalar(double value, Unit unit){
        this.value=unit.convertToSI(value);
        this.dimensions=unit.getDimensions();
    }

    ///Creates a Scalar object from value in SI units and dimensions. Only used internally
    private Scalar(double value, Dimensions dimensions){
        this.value=value;
        this.dimensions=dimensions;
    }

    public double getValueSI(){
        return value;
    }

    /// Gets value in Param units
    /// @throws RuntimeException unitError if dimensions do not match
    public double getValue(Unit unit){
        if(dimensions.equals(unit.getDimensions())){
            return unit.convertFromSI(value);
        }
        else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot convert unit of dimensions %s and SI base units %s to new dimensions %s and SI base units %s",dimensions.toString(), dimensions.SIBaseUnitStr(), dimensions.toString(), dimensions.SIBaseUnitStr()));
        }
    }

    public Dimensions getDimensions(){
        return dimensions;
    }

    public Scalar pow(double exponent){
        return new Scalar(Math.pow(value,exponent), dimensions.pow(exponent));
    }
    ///Adds two scalars, if dimensions are the same
    /// @throws RuntimeException unitError if dimensions do not match
    public Scalar add(Scalar scalar){
        if(dimensions.equals(scalar.getDimensions())){
            return new Scalar(value+scalar.getValueSI(), dimensions);
        }
        else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot add unit of dimensions %s and SI base units %s to new dimensions %s and SI base units %s",dimensions.toString(), dimensions.SIBaseUnitStr(), dimensions.toString(), dimensions.SIBaseUnitStr()));
        }
    }

    ///Subtracts two scalars, if dimensions are the same
    /// @throws RuntimeException unitError if dimensions do not match
    public Scalar sub(Scalar scalar){
        if(dimensions.equals(scalar.getDimensions())){
            return new Scalar(value-scalar.getValueSI(), dimensions);
        }
        else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot subtract unit of dimensions %s and SI base units %s to new dimensions %s and SI base units %s",dimensions.toString(), dimensions.SIBaseUnitStr(), dimensions.toString(), dimensions.SIBaseUnitStr()));
        }
    }

    public Scalar multiply(Scalar scalar){
        return new Scalar(value*scalar.getValueSI(), dimensions.multiply(scalar.getDimensions()));
    }

    public Scalar div(Scalar scalar){
        return new Scalar(value/scalar.getValueSI(), dimensions.div(scalar.getDimensions()));
    }

    public Scalar multiply(double factor){
        return new Scalar(value*factor, dimensions);
    }

    public Scalar div(double factor){
        return new Scalar(value/factor, dimensions);
    }

    public Boolean equals(Scalar scalar){
        return dimensions==scalar.dimensions && value==scalar.value;
    }

    /// @throws RuntimeException unitError if dimensions do not match
    public Boolean greaterThan(Scalar scalar){
        if(dimensions.equals(scalar.getDimensions())){
            return value>scalar.value;
        }
        else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot compare unit of dimensions %s and SI base units %s to new dimensions %s and SI base units %s",dimensions.toString(), dimensions.SIBaseUnitStr(), dimensions.toString(), dimensions.SIBaseUnitStr()));
        }
    }

    /// @throws RuntimeException unitError if dimensions do not match
    public Boolean greaterThanOrEqual(Scalar scalar){
        if(dimensions.equals(scalar.getDimensions())){
            return value>=scalar.value;
        }
        else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot compare unit of dimensions %s and SI base units %s to new dimensions %s and SI base units %s",dimensions.toString(), dimensions.SIBaseUnitStr(), dimensions.toString(), dimensions.SIBaseUnitStr()));
        }
    }

    /// @throws RuntimeException unitError if dimensions do not match
    public Boolean lessThan(Scalar scalar){
        if(dimensions.equals(scalar.getDimensions())){
            return value<scalar.value;
        }
        else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot compare unit of dimensions %s and SI base units %s to new dimensions %s and SI base units %s",dimensions.toString(), dimensions.SIBaseUnitStr(), dimensions.toString(), dimensions.SIBaseUnitStr()));
        }
    }

    /// @throws RuntimeException unitError if dimensions do not match
    public Boolean lessThanOrEqual(Scalar scalar){
        if(dimensions.equals(scalar.getDimensions())){
            return value<=scalar.value;
        }
        else{
            throw new RuntimeException(String.format("unitError: Dimension mismatch \nCannot compare unit of dimensions %s and SI base units %s to new dimensions %s and SI base units %s",dimensions.toString(), dimensions.SIBaseUnitStr(), dimensions.toString(), dimensions.SIBaseUnitStr()));
        }
    }
}