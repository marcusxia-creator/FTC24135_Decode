package org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.Examples;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.maxAccel;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.DimlessVector;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

import java.util.Arrays;
import java.util.List;

/// An example IceWaddler drive train object, built on our robot hardwaremap.<br>
/// If also using a hardwaremap, change class and motor names to match<br>
/// If not using a hardware map, modify the constructor to input and store the four motors as individual parameters
@Config
public class ExampleDriveTrain implements IWDriveTrain {
    public static double k_v=8.8;
    public static double angk_v=3;
    public static double k_a=1.5;
    public static double angk_a=5;
    public static double maxAllowableLinPower=11;
    public static Scalar wheelPivotRadius       = new Scalar(10, in); //The distance between the pivot point and each of the wheels, or half the length of the diagonal
    RobotHardware robot;
    ///In this implimentation, the constructor simply stores our hardware map
    public ExampleDriveTrain(RobotHardware robot){
        this.robot=robot;
    }

    @Override
    public void run(DimlessVector linPower, double angPower) {
        if(linPower.mag()>maxAllowableLinPower){
            linPower=linPower.unitVector().multi(maxAllowableLinPower);
        }
        double forward=linPower.getY();
        double strafe=linPower.getX();
        double rot=angPower;

        runVoltage(
                forward+strafe+rot,
                forward-strafe+rot,
                forward-strafe-rot,
                forward+strafe-rot
        );
    }

    @Override
    public void run(Velocity currentVelocity, Acceleration targetAcceleration, NormalizedAngle currentHeading){
                DimlessVector linPower=currentVelocity.getLinVel().getDimlessVector().multi(k_v).add(targetAcceleration.getLinAcc().getDimlessVector().multi(k_a)).rotateBy(currentHeading.multiply(-1));
        double angPower=angk_v*currentVelocity.getAngVel().getValueSI()+angk_a*targetAcceleration.getAngAcc().getValueSI();
        run(linPower,angPower);
    }

    @Override
    public void runPower(double FL_Power, double BL_Power, double FR_Power, double BR_Power) {
        robot.frontLeftMotor.setPower(FL_Power);
        robot.backLeftMotor.setPower(BL_Power);
        robot.frontRightMotor.setPower(FR_Power);
        robot.backRightMotor.setPower(BR_Power);
    }

    public void runVoltage(double FL_Voltage, double BL_Voltage, double FR_Voltage, double BR_Voltage){
        double voltage=robot.voltageSensor.getVoltage();
        runPower(FL_Voltage/voltage,BL_Voltage/voltage,FR_Voltage/voltage,BR_Voltage/voltage);
    }
}
