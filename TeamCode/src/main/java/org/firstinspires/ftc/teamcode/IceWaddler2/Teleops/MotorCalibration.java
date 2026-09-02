package org.firstinspires.ftc.teamcode.IceWaddler2.Teleops;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActionParallel;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActivatableAction;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.threeten.bp.Instant;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Motor Power Calibration", group="IceWaddler")
@Config
public class MotorCalibration extends OpMode {
    RobotHardware robot = new RobotHardware(hardwareMap);

    IceWaddler waddler;

    //logging
    String filepath;
    private BufferedWriter csvWriter;

    Scalar distPerTick;

    FtcDashboard dashboard;

    Action rootAction;

    double power;
    double velocity;
    double acceleration;
    boolean slipping;

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.driveTrain.init();

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");

        waddler = new IceWaddler(robot.driveTrain, robot.localizer);

        waddler.init(new Position(new Vector(0, 0, m), new NormalizedAngle(0, deg)),
                true);

        filepath = String.format("../sdcard/IceWaddler_Tuning/%s.csv", Instant.now());

        try {
            csvWriter = new BufferedWriter(new FileWriter(filepath));

            csvWriter.write("Power,Vel,Acc");
            csvWriter.newLine();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        rootAction = new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,
                waddler.new PowerDrive(() -> (double) gamepad1.right_stick_y),
                new ActivatableAction(() -> gamepad1.a, () -> gamepad1.b, new logger()),
                new telemetryDriver());
    }

    @Override
    public void init_loop() {
        waddler.update();
        if(robot.frontLeftMotor.getVelocity()!=0){
            distPerTick=waddler.getCurrentSituation().getPosition().getY().div(robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("Current conversion Factor Estimation", String.format("%f m/tick", distPerTick.getValue(m)));
        }
    }

    @Override
    public void start() {
        rootAction.init();
    }

    @Override
    public void loop() {
        rootAction.loop();
    }

    @Override
    public void stop() {
        rootAction.shutdown();

        try {
            csvWriter.flush();
            csvWriter.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    class logger implements Action {
        public logger(){}

        @Override
        public void loop() {
            if(!slipping) {
                try {
                    csvWriter.write(String.format("%f9,%f9,%f9",
                            power, velocity, acceleration));
                    csvWriter.newLine();
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    class telemetryDriver implements Action{
        public telemetryDriver(){}

        @Override
        public void loop() {
            power=gamepad1.right_stick_y;
            velocity=robot.frontLeftMotor.getVelocity();
            acceleration=waddler.getCurrentSituation().getAcceleration().getX().getValueSI();
            slipping=waddler.getCurrentSituation().getVelocity().getY().div(new Scalar(velocity,perSecond)).lessThanOrEqual(distPerTick.multiply(0.9));

            telemetry.addLine("Currently logging to " + filepath + ", press B to stop logging");
            telemetry.addData("Power", power);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Acceleration", acceleration);
            telemetry.addData("Slipping",slipping);
            telemetry.update();

            dashboard.sendTelemetryPacket(waddler.drawField());
        }
    }
}
