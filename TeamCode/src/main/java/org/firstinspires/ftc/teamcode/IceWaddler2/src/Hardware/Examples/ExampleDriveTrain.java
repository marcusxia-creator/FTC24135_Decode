package org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.Examples;

import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.maxAccel;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import java.util.Arrays;
import java.util.List;

/// An example IceWaddler drive train object, built on our robot hardwaremap.<br>
/// If also using a hardwaremap, change class and motor names to match<br>
/// If not using a hardware map, modify the constructor to input and store the four motors as individual parameters
public class ExampleDriveTrain implements IWDriveTrain {
    RobotHardware robot;
    ///In this implimentation, the constructor simply stores our hardware map
    public ExampleDriveTrain(RobotHardware robot){
        this.robot=robot;
    }

    @Override
    public void init(){
        /// Our drivetrain does not need an init, as reversing is already handled in hardware init, so this method is left blank
    }

    @Override
    public void runPowers(double FL_Power, double BL_Power, double FR_Power, double BR_Power) {
        robot.frontLeftMotor.setPower(FL_Power);
        robot.backLeftMotor.setPower(BL_Power);
        robot.frontRightMotor.setPower(FR_Power);
        robot.backRightMotor.setPower(BR_Power);
    }

    @Override
    public List<Double> getVelocities() {
        return Arrays.asList(
                robot.frontLeftMotor.getVelocity(), //Velocity of front left motor, in ticks/s
                robot.backLeftMotor.getVelocity(),  //Velocity of back left motor, in ticks/s
                robot.frontRightMotor.getVelocity(),//Velocity of front left motor, in ticks/s
                robot.backLeftMotor.getVelocity()   //Velocity of front left motor, in ticks/s
        );
    }

    @Override
    public double powerController(Scalar Accel, double motorVel) {
        return Accel.div(maxAccel).getValueSI(); //Temporary Placeholder
    }

    @Override
    public void runAccel(Scalar FL_Accel, Scalar BL_Accel, Scalar FR_Accel, Scalar BR_Accel) {
        robot.frontLeftMotor.setPower(powerController(FL_Accel,robot.frontLeftMotor.getVelocity()));
        robot.backLeftMotor.setPower(powerController(BL_Accel,robot.backLeftMotor.getVelocity()));
        robot.frontRightMotor.setPower(powerController(FR_Accel,robot.frontRightMotor.getVelocity()));
        robot.backRightMotor.setPower(powerController(BR_Accel,robot.backRightMotor.getVelocity()));
    }
}
