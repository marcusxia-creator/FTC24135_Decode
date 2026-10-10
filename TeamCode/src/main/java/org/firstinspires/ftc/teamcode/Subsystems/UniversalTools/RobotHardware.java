package org.firstinspires.ftc.teamcode.Subsystems.UniversalTools;


import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.Examples.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;

public class RobotHardware{
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx intakeMotor;
    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;

    public ExampleDriveTrain driveTrain;
    public goBildaOdoComputer localizer;
    public VoltageSensor voltageSensor;

    public HardwareMap hardwareMap;
    public RobotHardware(HardwareMap hardwareMap) {
       this.hardwareMap=hardwareMap;
    }
    public void init(){
        ///Drive
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // set motor mode
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); //set motor mode
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // set motor mode
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // set motor mode

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ///Intake
        //intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");

        //intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        //Icewaddler
        driveTrain=new ExampleDriveTrain(this);
        localizer=new goBildaOdoComputer(pinpoint,new Scalar(40,mm),new Scalar(-200,mm), GoBildaOdometryPods.goBILDA_SWINGARM_POD, EncoderDirection.REVERSED, EncoderDirection.REVERSED);
        voltageSensor=hardwareMap.get(VoltageSensor.class, "Control Hub");
    }
}
