package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.Dimension.Angular;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.Dimension.Spacial;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Dimensions.Dimension.Temporal;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Dimensions {
    enum Dimension{
        Spacial("m"),
        Angular("rad"),
        Temporal("s");

        private final String SIBaseUnit;

        Dimension(String SIBaseUnit){this.SIBaseUnit=SIBaseUnit;}
    }

    Map<Dimension,Double> powers;

    /// Creates a Dimensions storage object, which stores and allows operations with exponents of Spacial, Angular, and Temporal dimensions<br>
    /// @param spacialDims The number of spacial dimensions
    /// @param angularDims The number of angular dimensions
    /// @param temporalDims The number of temporal(time) dimensions
    public Dimensions(double spacialDims, double angularDims, double temporalDims){
        powers=new HashMap<>();
        powers.put(Spacial,spacialDims);
        powers.put(Angular,angularDims);
        powers.put(Temporal,temporalDims);
    }

    private Dimensions(Map<Dimension,Double> powers){
        this.powers=powers;
    }

    public double getSpacialDims() {
        return powers.get(Spacial);
    }

    public double getAngularDims() {
        return powers.get(Angular);
    }

    public double getTemporalDims() {
        return powers.get(Temporal);
    }

    public Dimensions pow(double exponent){
        Map<Dimension,Double> newPowers=new HashMap<>();
        powers.forEach((key,value)->newPowers.put(key,value*exponent));
        return new Dimensions(newPowers);
    }

    public Dimensions multiply(Dimensions dim){
        Map<Dimension,Double> newPowers=new HashMap<>();
        powers.forEach((key,value)->newPowers.put(key,value+dim.powers.get(key)));
        return new Dimensions(newPowers);
    }

    public Dimensions div(Dimensions dim){
        return multiply(dim.pow(-1));
    }

    public boolean equals(Dimensions dim){
        return powers.equals(dim.powers);
    }

    @Override
    public String toString(){
        List<String> list = new ArrayList<>();
        powers.forEach((key,value)->{
            if(value==1){
                list.add(key.name());
            }
            else if(value!=0){
                list.add(key.name()+"^"+value);
            }
        });

        if(list.isEmpty()){
            return "Dimensionless";
        }else{
            return String.join(", ",list);
        }
    }

    ///@return A string representing the SI base units with dimensions
    public String SIBaseUnitStr(){
        List<String> list = new ArrayList<>();
        powers.forEach((key,value)->{
            if(value==1){
                list.add(key.SIBaseUnit);
            }
            else if(value!=0){
                list.add(key.SIBaseUnit+"^"+value);
            }
        });

        if(list.isEmpty()){
            return "(Dimensionless)";
        }else{
            return String.join(" ",list);
        }
    }

    ///@return A unit object of the SI base unit with dimensions
    public Unit SIBaseUnit(){
        return new Unit(1, this);
    }

    //Premade Dimensions
    public static Dimensions length=new Dimensions(1,0,0);
    public static Dimensions angle=new Dimensions(0,1,0);
    public static Dimensions time=new Dimensions(0,0,1);
    public static Dimensions rate=new Dimensions(0,0,-1);
    public static Dimensions velocity=new Dimensions(1,0,-1);
    public static Dimensions angVelocity=new Dimensions(0,1,-1);
    public static Dimensions acceleration=new Dimensions(1,0,-2);
    public static Dimensions angAcceleration=new Dimensions(0,1,-2);
    public static Dimensions dimensionless=new Dimensions(0,0,0);
}