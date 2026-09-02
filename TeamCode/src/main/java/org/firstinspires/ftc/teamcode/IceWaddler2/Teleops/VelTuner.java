package org.firstinspires.ftc.teamcode.IceWaddler2.Teleops;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp(name="Velocity Tuner", group="IceWaddler")
@Config
public class VelTuner extends OpMode {
    RobotHardware robot;
    IceWaddler waddler;
    FtcDashboard dashboard;

    public static Scalar linVelFactor=new Scalar(2.5,metersPerSecond);
    public static Scalar angVelFactor=new Scalar(2,radiansPerSecond);

    Action rootAction;

    @Override
    public void init(){
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(new Position(new Vector(0,0,m),new NormalizedAngle(0,deg)),false);

        dashboard=FtcDashboard.getInstance();

        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        rootAction=new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,
                waddler.new VelDrive(()->new Velocity(new Vector(linVelFactor.multiply(gamepad1.right_stick_x),linVelFactor.multiply(gamepad1.right_stick_y)),
                        angVelFactor.multiply(gamepad1.left_stick_x))),
                new telemetryDriver()
        );
    }

    @Override
    public void start() {
        rootAction.init();
    }

    @Override
    public void loop(){
        rootAction.loop();
    }

    @Override
    public void stop() {
        rootAction.shutdown();
    }

    class telemetryDriver implements Action {
        public telemetryDriver(){}

        @Override
        public void loop() {
            telemetry.addData("1.Current x vel",waddler.getCurrentSituation().getVelocity().getX().getValueSI());
            telemetry.addData("1.Current y vel",waddler.getCurrentSituation().getVelocity().getY().getValueSI());
            telemetry.addData("1.Current ang vel",waddler.getCurrentSituation().getVelocity().getAngVel().getValueSI());

            telemetry.addData("2.Target x vel", waddler.getTargetSituation().getVelocity().getX().getValueSI());
            telemetry.addData("2.Target y vel", waddler.getTargetSituation().getVelocity().getY().getValueSI());
            telemetry.addData("2.Target ang vel", waddler.getTargetSituation().getVelocity().getAngVel().getValueSI());

            telemetry.addData("3.Target x acc", waddler.getTargetSituation().getAcceleration().getX().getValueSI());
            telemetry.addData("3.Target y acc", waddler.getTargetSituation().getAcceleration().getY().getValueSI());
            telemetry.addData("3.Target ang acc", waddler.getTargetSituation().getAcceleration().getAngAcc().getValueSI());
        }
    }
}
