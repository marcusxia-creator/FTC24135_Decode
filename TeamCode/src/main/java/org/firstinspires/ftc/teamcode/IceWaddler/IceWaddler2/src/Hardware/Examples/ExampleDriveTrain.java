package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.Examples;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import java.util.Arrays;
import java.util.Collections;
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

    public void init(){
        /// Our drivetrain does not need an init, as reversing is already handled in hardware init, so this method is left blank
    }

    public void writePowers(double FL_Power, double BL_Power, double FR_Power, double BR_Power) {
        robot.frontLeftMotor.setPower(FL_Power);
        robot.backLeftMotor.setPower(BL_Power);
        robot.frontRightMotor.setPower(FR_Power);
        robot.backRightMotor.setPower(BR_Power);
    }

    public List<Double> getVelocities() {
        return Arrays.asList(
                robot.frontLeftMotor.getVelocity(), //Velocity of front left motor, in ticks/s
                robot.backLeftMotor.getVelocity(),  //Velocity of back left motor, in ticks/s
                robot.frontRightMotor.getVelocity(),//Velocity of front left motor, in ticks/s
                robot.backLeftMotor.getVelocity()   //Velocity of front left motor, in ticks/s
        );
    }
}
