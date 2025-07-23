package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled

@Config
@TeleOp (name = "PIDF Arm", group = "org.firstinspires.ftc.teamcode")
public class PIDFSlideOpMode extends OpMode{
    private GamepadEx gamepadCo1, gamepadCo2;
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static double target = 0;    /// travel distance

    public final double ticksPerMM = RobotActionConfig.TICKS_PER_MM_SLIDES;

    public final double maxTicks = RobotActionConfig.deposit_Slide_Highbasket_Pos*RobotActionConfig.TICKS_PER_MM_SLIDES;

    RobotHardware robot;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    @Override
    public void init (){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);


        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
    }

    @Override
    public void loop (){
        controller.setPID(p,i,d);
        int slidePos = (robot.liftMotorLeft.getCurrentPosition()+robot.liftMotorRight.getCurrentPosition())/2;

        if (gamepadCo1.getButton(X) && isButtonDebounced()){
            target =50;
        }

        if (gamepadCo1.getButton(Y) && isButtonDebounced()){
            target =500;
        }
        // Normalize if requested
        double normalizedTarget = target * ticksPerMM/maxTicks;

        double normalizedSlidePos = slidePos/maxTicks;

        double pid = controller.calculate(normalizedSlidePos, normalizedTarget);

        double ff = target * ticksPerMM > slidePos ? f : 0;

        double power = pid + ff;

        power = Range.clip(power,-1.0,1.0);


        robot.liftMotorLeft.setPower(power);
        robot.liftMotorRight.setPower(power);

        telemetry.addData("target in tick", target * ticksPerMM);
        telemetry.addData("position in tick", slidePos);
        telemetry.addLine("------------------------");
        telemetry.addData("Normalized target in %", normalizedTarget);
        telemetry.addData("position in %", normalizedSlidePos);
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
