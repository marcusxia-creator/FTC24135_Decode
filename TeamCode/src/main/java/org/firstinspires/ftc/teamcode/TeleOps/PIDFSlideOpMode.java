package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Config
@TeleOp (name = "PIDF Slide Ops", group = "org.firstinspires.ftc.teamcode")
public class PIDFSlideOpMode extends OpMode{
    private GamepadEx gamepadCo1, gamepadCo2;
    //private PIDController controller;         // FTClib PIDcontroller
    private SlidesPIDControl controller;

    public static double p = 5.0, i = 0, d = 0.05;
    public static double f = 0.12;

    public static double target = 0;    /// travel distance

    public final double ticksPerMM = RobotActionConfig.TICKS_PER_MM_SLIDES;

    public final double maxTicks = RobotActionConfig.deposit_Slide_Highbasket_Pos*RobotActionConfig.TICKS_PER_MM_SLIDES;

    public static double power =0;
    public static double ff =0;
    public static double measurement = 0;

    private RobotHardware robot;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    @Override
    public void init (){
        //controller = new PIDController(p, i, d);
        controller = new SlidesPIDControl(robot,p,i,d,ff,maxTicks,ticksPerMM);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);
        /**
         *set motor Run Mode
        for (DcMotor m: new DcMotor[]{robot.liftMotorLeft,robot.liftMotorRight })
        {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/



        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        telemetry.addData("VS Left Encoder", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("VS Right Encoder", robot.liftMotorRight.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop (){
        //controller.setPID(p,i,d);
        int slidePos = robot.liftMotorRight.getCurrentPosition();

        if (gamepadCo1.getButton(X) && isButtonDebounced()){
            target =5; // mm
        }

        if (gamepadCo1.getButton(Y) && isButtonDebounced()){
            target =650;
        }

        double normalizedTarget = target * ticksPerMM/maxTicks;

        double normalizedSlidePos = slidePos/maxTicks;

        ///simple linear line
        ///double pid = controller.calculate(normalizedSlidePos, normalizedTarget);

        ///double ff = target * ticksPerMM > slidePos ? f : 0;

        ///double power = pid + ff;

        ///power = Range.clip(power,-1.0,1.0);

        /////robot.liftMotorLeft.setPower(power);
        ///robot.liftMotorRight.setPower(power);


        /// set PID target point
        controller.setTargetMM(target);
        controller.update();
        //motorDrive(power);

        telemetry.addData("target mm", target);
        telemetry.addData("target in tick", target * ticksPerMM);
        telemetry.addData("position in tick", slidePos);
        telemetry.addLine("------------------------");
        telemetry.addData("Normalized target in %", normalizedTarget);
        telemetry.addData("position in %", normalizedSlidePos);
        telemetry.addData("Measurement = position in %", measurement);
        telemetry.addLine("------------------------");
        telemetry.addData("Motor Power", power);
        telemetry.addData("Feedfoward F", ff);

        telemetry.update();
    }

    ///- Button Debounce Helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
    /**
     *
    ///Handler - Set PID targetTick directly
    private void setTargetTick(double targetTicks){
        if (maxTicks>0)
        {
            controller.setSetPoint(targetTicks / maxTicks);
        }
        else {
            controller.setSetPoint(targetTicks);
        }
    }
    ///Handler - Set PID target based on MM; convert MM to Ticks and Set the Ticks in setTargetTick.
    private void setTargetMM(double mm){
        setTargetTick(mmToTicks(mm));
    }

    /// helper - Convert linear inches to encoder ticks; adjust counts and diameter
    private double mmToTicks(double mm) {
        return mm * RobotActionConfig.TICKS_PER_MM_SLIDES;
    }

    /// Handler - Update PID calculation for power
    private void PIDUpdate( ){
        double avgPos  = robot.liftMotorRight.getCurrentPosition();
        ///double avgPos = rightPos;

        // Normalize if requested
        measurement = (maxTicks > 0) ? avgPos / maxTicks : avgPos;

        // Compute PID output
        double PIDpower = controller.calculate(measurement);
        // Compute feedforward f
        ff = target * ticksPerMM > avgPos ? f : 0;
        // Compute motor power
        power =PIDpower+ff;
        //clip power to -1 to 1
        power = Range.clip(power, -1.0, 1.0);
    }

    ///Handler - drive motor
    private void motorDrive(double power){
        // Apply to both motors
        //robot.liftMotorLeft.setPower(power);
        robot.liftMotorRight.setPower(power);
    }
     */
}
