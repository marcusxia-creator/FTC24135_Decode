package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.Teleops;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.Examples.ExampleDriveTrain;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.Examples.OTOS;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWLocalizer;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Vector;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;


@TeleOp(name="Odo Tester", group="IceWaddler")
public class OdoTester extends OpMode {
    RobotHardware robot;
    IceWaddler waddler;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        waddler = new IceWaddler(robot.driveTrain, robot.localizer);

        waddler.init(IceWaddler.CONTROLMODE.STBY, new Position(new Vector(0, 0, m), new NormalizedAngle(0, deg)), true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        waddler.loop();
        telemetry.addData("xPos", waddler.getCurrentSituation().getPosition().getX().getValueSI());
        telemetry.addData("yPos", waddler.getCurrentSituation().getPosition().getY().getValueSI());
        telemetry.addData("Heading", waddler.getCurrentSituation().getPosition().getAngPos().getValue(deg));

        telemetry.addData("xVel", waddler.getCurrentSituation().getVelocity().getX().getValueSI());
        telemetry.addData("yVel", waddler.getCurrentSituation().getVelocity().getY().getValueSI());
        telemetry.addData("AngVel", waddler.getCurrentSituation().getVelocity().getAngVel().getValue(degreesPerSecond));


        telemetry.addData("xAcc", waddler.getCurrentSituation().getAcceleration().getX().getValueSI());
        telemetry.addData("yAcc", waddler.getCurrentSituation().getAcceleration().getY().getValueSI());
        telemetry.addData("AngAcc", waddler.getCurrentSituation().getAcceleration().getAngAcc().getValue(degreesPerSecondSquared));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
