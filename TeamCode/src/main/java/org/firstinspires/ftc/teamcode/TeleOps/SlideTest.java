package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.SliceMatrixF;

@TeleOp(name = "slide test", group = "org.firstinspires.ftc.teamcode")
public class SlideTest extends OpMode{
    public DcMotorEx liftMotorLeft;
    public DcMotorEx liftMotorRight;
    public Servo servo1;
    public Servo servo2;
    public Servo servo3;
    public Servo servo4;

    public double servoposition;

    public GamepadEx gamepad_1;
    public GamepadEx gamepad_2;
    int currentposition;
    public int deltaposition = 50;
    private final ElapsedTime debounceTimer = new ElapsedTime();




    public void init(){
        liftMotorLeft = hardwareMap.get(DcMotorEx.class,"VS_Left_Motor");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "VS_Right_Motor");
        servo1 = hardwareMap.get(Servo.class, "Servo_1");
        servo2 = hardwareMap.get(Servo.class, "Servo_2");
        servo3 = hardwareMap.get(Servo.class, "Servo_3");
        servo4 = hardwareMap.get(Servo.class, "Servo_4");
        //liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotorLeft.setTargetPosition(100);
        //liftMotorRight.setTargetPosition(50);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorLeft.setPower(0.2);
        //liftMotorRight.setPower(0.2);
        servo1.setPosition(0);
        servo2.setPosition(0);
        servo3.setPosition(0);
        servo4.setPosition(0);

        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
    }

    @Override
    public void loop(){

        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_UP) && debounceTimer.seconds()>0.25){
            currentposition = liftMotorLeft.getCurrentPosition();
            //liftMotorLeft.setTargetPosition(Range.clip(currentposition + deltaposition, 50, 1000));
            liftMotorRight.setTargetPosition(Range.clip(currentposition + deltaposition, 50, 1000));
            //liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //liftMotorLeft.setPower(0.2);
            liftMotorRight.setPower(0.2);
            debounceTimer.reset();
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_DOWN) && debounceTimer.seconds()>0.25){
            currentposition = liftMotorLeft.getCurrentPosition();
            //liftMotorLeft.setTargetPosition(Range.clip(currentposition - deltaposition, 50, 1000));
            liftMotorRight.setTargetPosition(Range.clip(currentposition - deltaposition, 50, 1000));
            //liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //liftMotorLeft.setPower(0.2);
            liftMotorRight.setPower(0.2);
            debounceTimer.reset();
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && debounceTimer.seconds()>0.25){
            servoposition = servo1.getPosition();
            servoposition += 0.01;
            servo1.setPosition(Range.clip(servoposition,0,1));
            debounceTimer.reset();
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && debounceTimer.seconds()>0.25){
            servoposition = servo1.getPosition();
            servoposition -= 0.01;
            servo1.setPosition(Range.clip(servoposition,0,1));
            debounceTimer.reset();
        }
        if (gamepad_1.getButton(GamepadKeys.Button.X) && debounceTimer.seconds()>0.25){
            servoposition = servo2.getPosition();
            servoposition += 0.01;
            servo2.setPosition(Range.clip(servoposition,0,1));
            debounceTimer.reset();
        }
        if (gamepad_1.getButton(GamepadKeys.Button.Y) && debounceTimer.seconds()>0.25){
            servoposition = servo2.getPosition();
            servoposition -= 0.01;
            servo2.setPosition(Range.clip(servoposition,0,1));
            debounceTimer.reset();
        }
        if (gamepad_1.getButton(GamepadKeys.Button.A) && debounceTimer.seconds()>0.25){
            servoposition = servo3.getPosition();
            servoposition += 0.01;
            servo3.setPosition(Range.clip(servoposition,0,1));
            debounceTimer.reset();
        }
        if (gamepad_1.getButton(GamepadKeys.Button.B) && debounceTimer.seconds()>0.25){
            servoposition = servo3.getPosition();
            servoposition -= 0.01;
            servo3.setPosition(Range.clip(servoposition,0,1));
            debounceTimer.reset();
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && debounceTimer.seconds()>0.25){
            servoposition = servo4.getPosition();
            servoposition += 0.01;
            servo4.setPosition(Range.clip(servoposition,0,1));
            debounceTimer.reset();
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && debounceTimer.seconds()>0.25){
            servoposition = servo4.getPosition();
            servoposition -= 0.01;
            servo4.setPosition(Range.clip(servoposition,0,1));
            debounceTimer.reset();
        }

        telemetry.addData("current slide position", currentposition);
        telemetry.update();

    }
    public void stop () {
        //liftMotorLeft.setPower(0);
        liftMotorRight.setPower(0);
    }
}


