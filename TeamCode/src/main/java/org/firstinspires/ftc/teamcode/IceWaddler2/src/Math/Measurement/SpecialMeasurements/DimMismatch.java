package org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements;


import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions;

public class DimMismatch extends RuntimeException {
    public DimMismatch(Dimensions givenDims, String type) {
        super(String.format("unitError: Dimension mismatch \nCannot store scalar of dimensions %s and SI base units %s as a %s",givenDims.toString(), givenDims.SIBaseUnitStr(), type));
    }
}
