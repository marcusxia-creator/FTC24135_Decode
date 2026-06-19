package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.Teleops;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.Examples.ExampleDriveTrain;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.Examples.OTOS;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWLocalizer;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Vector;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@TeleOp(name="Motor Power Calibration", group="IceWaddler")
@Config
public class MotorCalibration extends LinearOpMode {
    public static Scalar lowPowerTestingRange=new Scalar(0.7, field);//Field length available for low speed calibration
    public static Scalar highPowerTestingRange=new Scalar(0.5, field);//Field length available for high speed calibration, recommended to be a bit shorter to leave margin for error

    RobotHardware robot = new RobotHardware(hardwareMap);

    IceWaddler waddler;
    boolean stopRequested=false;
    double power=0.1;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Motor Power Calibration will calibrate over "+lowPowerTestingRange.getValue(m)+" m at low speed," +
                "and "+highPowerTestingRange.getValue(m) +" m at high speed");
        telemetry.addLine("Start to begin low power calibration");
        telemetry.update();

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);

        waddler.init(IceWaddler.CONTROLMODE.STBY,
                new Position(new Vector(0,0,m), new NormalizedAngle(0, deg)),
                true);

        waddler.loop();

        waitForStart();

        // Low power calibration
        while (power<0.5&&opModeIsActive()) {
            robot.driveTrain.writePowers(power,power,power,power);
            while(waddler.getCurrentSituation().getPosition().getY().lessThanOrEqual(lowPowerTestingRange.multiply(0.5))&&opModeIsActive()){
                waddler.loop();
                checkStop();
                logData(false);
            }
            power=-power;
            robot.driveTrain.writePowers(power,power,power,power);
            while(waddler.getCurrentSituation().getVelocity().getY().greaterThan(new Scalar(0,metersPerSecond))&&opModeIsActive()){
                waddler.loop();
                checkStop();
                logData(false);
            }
            processStop();
            power=power-0.05;
            robot.driveTrain.writePowers(power,power,power,power);
            while(waddler.getCurrentSituation().getPosition().getY().greaterThanOrEqual(lowPowerTestingRange.multiply(0.5))&&opModeIsActive()){
                waddler.loop();
                checkStop();
                logData(true);
            }
            power=-power;
            robot.driveTrain.writePowers(power,power,power,power);
            while(waddler.getCurrentSituation().getVelocity().getY().lessThan(new Scalar(0,metersPerSecond))){
                waddler.loop();
                checkStop();
                logData(true);
            }
            processStop();
            power=power+0.05;
        }
        stopRequested=true;
        telemetry.addLine("Low speed calibration complete, start high speed calibration");
        processStop();
        waddler.init(IceWaddler.CONTROLMODE.STBY,
                new Position(new Vector(0,0,m), new NormalizedAngle(0, deg)),
                true);
        while (power<1&&opModeIsActive()) {
            robot.driveTrain.writePowers(power,power,power,power);
            while(waddler.getCurrentSituation().getPosition().getY().lessThanOrEqual(lowPowerTestingRange.multiply(0.5))&&opModeIsActive()){
                waddler.loop();
                checkStop();
                logData(false);
            }
            power=-power;
            robot.driveTrain.writePowers(power,power,power,power);
            while(waddler.getCurrentSituation().getVelocity().getY().greaterThan(new Scalar(0,metersPerSecond))&&opModeIsActive()){
                waddler.loop();
                checkStop();
                logData(false);
            }
            processStop();
            power=power-0.05;
            robot.driveTrain.writePowers(power,power,power,power);
            while(waddler.getCurrentSituation().getPosition().getY().greaterThanOrEqual(lowPowerTestingRange.multiply(0.5))&&opModeIsActive()){
                waddler.loop();
                checkStop();
                logData(true);
            }
            power=-power;
            robot.driveTrain.writePowers(power,power,power,power);
            while(waddler.getCurrentSituation().getVelocity().getY().lessThan(new Scalar(0,metersPerSecond))&&opModeIsActive()){
                waddler.loop();
                checkStop();
                logData(true);
            }
            processStop();
            power=power+0.05;
        }
    }

    void checkStop(){
        if(gamepad1.b||gamepad2.b){
            stopRequested=true;
        }
    }

    void processStop() {
        if (stopRequested) {
            waddler.zeroPower();
            telemetry.addLine("Stopped, press a to proceed");
            telemetry.update();
            while (!(gamepad1.a || gamepad2.a)) {
            }
            stopRequested = false;
        }
    }

    void logData(boolean reversed){
        telemetry.addData("Power", (reversed?-1:1)*power);
        telemetry.addData("Motor tick rate", (reversed?-1:1)*robot.frontLeftMotor.getVelocity());
        telemetry.addData("Acceleration", (reversed?-1:1)*waddler.getCurrentSituation().getAcceleration().getY().getValueSI());
        telemetry.update();
    }
}
