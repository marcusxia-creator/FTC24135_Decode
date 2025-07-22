package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled

@Config
@TeleOp (name = "PIDF Arm", group = "org.firstinspires.ftc.teamcode")
public class PIDFSlide extends OpMode{
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    public final double ticksPerDegree = 384.5/360;

    public final double maxTicks = RobotActionConfig.deposit_Slide_Highbasket_Pos*RobotActionConfig.TICKS_PER_MM_SLIDES;

    RobotHardware robot;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    @Override
    public void init (){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop (){
        controller.setPID(p,i,d);
        int slidePos = (robot.liftMotorLeft.getCurrentPosition()+robot.liftMotorRight.getCurrentPosition())/2;

        if (gamepad1.A && isButtonDebounced() )){
            target =50;
        }

        if (gamepad1.getbutton(GamepadKeys.Buttom.Y && isButtonDebounced())){
            target =600;
        }
        double pid = controller.calculate(norslidePos, target);

        double ff = Math.cos(Math.toRadians(target / ticksPerDegree)) * f;

        double power = pid + ff;

        robot.liftMotorLeft.setPower(power);
        robot.liftMotorRight.setPower(power);

        telemetry.addData("position", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }


}
