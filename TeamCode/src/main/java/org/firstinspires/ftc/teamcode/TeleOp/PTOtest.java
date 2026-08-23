package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="PTO Test", group = "Test")
@Config
public class PTOtest extends OpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;
    Servo servo;

    double power;
    boolean engaged;

    Gamepad gamepad;

    //Params
    public static double disengagedPos=0.29;
    public static double engagedPos=0.39;

    @Override
    public void init(){
        motor1=hardwareMap.get(DcMotorEx.class,"motor1");
        motor2=hardwareMap.get(DcMotorEx.class,"motor2");
        //motor1 uses encoder, motor2 doesn't
        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        power=0;

        servo=hardwareMap.get(Servo.class,"Servo");
        engaged=false;
        servo.setPosition(disengagedPos);

        gamepad=gamepad1;

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop(){
        //Gamepad controls
        power=gamepad.right_stick_y;
        if(gamepad.dpad_down){
            engaged=true;
        }
        if(gamepad.dpad_up){
            engaged=false;
        }

        motor1.setPower(power);
        motor2.setPower(power);
        servo.setPosition(engaged?engagedPos:disengagedPos);

        telemetry.addLine("1. Controls");
        telemetry.addData("1. Power",power);
        telemetry.addData("1. Engaged",engaged);
        telemetry.addLine("2. Motor Data");
        //telemetry.addData("2. MotorVel",motor1.getVelocity());
        telemetry.addData("2. Motor1Current",motor1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("2. Motor2Current",motor2.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
