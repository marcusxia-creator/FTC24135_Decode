package org.firstinspires.ftc.teamcode.IceWaddler2.Teleops;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActionParallel;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.ActivatableAction;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.threeten.bp.Instant;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="Motor Power Calibration", group="IceWaddler")
@Config
public class MotorCalibration extends OpMode {
    RobotHardware robot = new RobotHardware(hardwareMap);

    IceWaddler waddler;

    double power;

    //logging
    boolean logging;
    String filepath;
    private BufferedWriter csvWriter;

    FtcDashboard dashboard;

    Action rootAction;

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

        logging = false;
        filepath = String.format("../sdcard/FIRST/%s.csv", Instant.now());

        try {
            csvWriter = new BufferedWriter(new FileWriter(filepath));

            csvWriter.write("Power,Vel,Acc");
            csvWriter.newLine();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        rootAction = new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,
                waddler.new PowerDrive(() -> (double) gamepad1.right_stick_y),
                new ActivatableAction(() -> gamepad1.a, () -> gamepad1.b, new logger()));
    }

    @Override
    public void start() {
        rootAction.init();
    }

    public void loop() {
        rootAction.loop();
    }

    public void stop() {
        rootAction.shutdown();
    }

    class logger implements Action {
        public logger(){}

        @Override
        public void init() {

        }

        @Override
        public void loop() {
            power = gamepad1.right_stick_y;
            robot.driveTrain.writePowers(power, power, power, power);

            telemetry.addLine(logging ? "Currently logging to " + filepath + ", press B to stop logging" : "Currently not logging, press A to start logging");
            telemetry.addData("Power", power);
            telemetry.addData("Velocity", robot.frontLeftMotor.getVelocity());
            telemetry.addData("Acceleration", waddler.getCurrentSituation().getAcceleration().getY().getValueSI());
            telemetry.update();

            dashboard.sendTelemetryPacket(waddler.drawField());

            if (logging) {
                try {
                    csvWriter.write(String.format("%f9,%f9,%f9",
                            power, robot.frontLeftMotor.getVelocity(), waddler.getCurrentSituation().getAcceleration().getY().getValueSI()));
                    csvWriter.newLine();
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }

        @Override
        public boolean finished() {
            return false;
        }

        @Override
        public void shutdown() {
            try {
                csvWriter.flush();
                csvWriter.close();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
