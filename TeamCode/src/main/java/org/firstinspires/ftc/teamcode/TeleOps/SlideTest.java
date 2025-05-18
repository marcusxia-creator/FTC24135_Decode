package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.SliceMatrixF;

@TeleOp(name = "slide test", group = "org.firstinspires.ftc.teamcode")
public class SlideTest {
    public DcMotorEx liftMotorLeft;
    public DcMotorEx liftMotorRight;
    public GamepadEx gamepad_1;
    int currentposition;
    int deltaposition;
    private final ElapsedTime debounceTimer = new ElapsedTime();

    public void init(HardwareMap hardwareMap){
        liftMotorLeft = hardwareMap.get(DcMotorEx.class,"VS_Left_Motor");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "VS_Right_Motor");
    }
    public void init(){
        liftMotorLeft.setTargetPosition(50);
        liftMotorRight.setTargetPosition(50);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorLeft.setPower(0.2);
        liftMotorRight.setPower(0.2);
    }
    public void SlideTestLoop(){
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && debounceTimer.seconds()>0.25){
            currentposition = liftMotorLeft.getCurrentPosition();

            liftMotorLeft.setTargetPosition(Range.clip(currentposition + deltaposition, 50, 1000));
            liftMotorRight.setTargetPosition(Range.clip(currentposition + deltaposition, 50, 1000));
            liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorLeft.setPower(0.2);
            liftMotorRight.setPower(0.2);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && debounceTimer.seconds()>0.25){
            currentposition = liftMotorLeft.getCurrentPosition();

            liftMotorLeft.setTargetPosition(Range.clip(currentposition - deltaposition, 50, 1000));
            liftMotorRight.setTargetPosition(Range.clip(currentposition - deltaposition, 50, 1000));
            liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorLeft.setPower(0.2);
            liftMotorRight.setPower(0.2);
        }
    }
}


