package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware{
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx leftLiftMotor;
    public DcMotorEx rightLiftMotor;

    public HardwareMap hardwareMap;
    public RobotHardware(HardwareMap hardwareMap) {
       this.hardwareMap=hardwareMap;
    }
    public void init(){
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");
        ///Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");
        ///Lift
        leftLiftMotor = hardwareMap.get(DcMotorEx.class,"Left_Lift_Motor");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "Right_Lift_Motor");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //set motor mode
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}
