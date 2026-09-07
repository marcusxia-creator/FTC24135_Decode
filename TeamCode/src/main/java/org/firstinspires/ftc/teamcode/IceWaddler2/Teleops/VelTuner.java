package org.firstinspires.ftc.teamcode.IceWaddler2.Teleops;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.*;
import org.firstinspires.ftc.teamcode.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp(name="Velocity Tuner", group="IceWaddler")
@Config
public class VelTuner extends ScheduledOpMode {
    RobotHardware robot;
    IceWaddler waddler;
    FtcDashboard dashboard;

    public static Scalar linVelFactor=new Scalar(2.5,metersPerSecond);
    public static Scalar angVelFactor=new Scalar(5,radiansPerSecond);

    Velocity getJoystickCommandVel(){
        return new Velocity(new Vector(linVelFactor.multiply(gamepad1.right_stick_x),linVelFactor.multiply(-gamepad1.right_stick_y)),
                angVelFactor.multiply(gamepad1.left_stick_x));
    }

    @Override
    public void init(){
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(new Position(new Vector(0,0,m),new NormalizedAngle(0,deg)),false);

        dashboard=FtcDashboard.getInstance();

        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        rootAction=new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,
                new ActionSwitch(()->gamepad1.b?1:0, waddler.new VelDrive(this::getJoystickCommandVel),
                        waddler.new Idle()),
                new telemetryDriver()
        );
    }

    class telemetryDriver implements Action {
        public telemetryDriver(){}

        @Override
        public void loop() {
            telemetry.addData("Ticktime",waddler.getTickTime().getValueSI());

            telemetry.addData("1.Current x vel",waddler.getCurrentSituation().getVelocity().getX().getValueSI());
            telemetry.addData("1.Current y vel",waddler.getCurrentSituation().getVelocity().getY().getValueSI());
            telemetry.addData("1.Current ang vel",waddler.getCurrentSituation().getVelocity().getAngVel().getValueSI());

            telemetry.addData("2.Target x vel", waddler.getTargetSituation().getVelocity().getX().getValueSI());
            telemetry.addData("2.Target y vel", waddler.getTargetSituation().getVelocity().getY().getValueSI());
            telemetry.addData("2.Target ang vel", waddler.getTargetSituation().getVelocity().getAngVel().getValueSI());

            telemetry.addData("3.Current x acc", waddler.getCurrentSituation().getAcceleration().getX().getValueSI());
            telemetry.addData("3.Current y acc", waddler.getCurrentSituation().getAcceleration().getY().getValueSI());
            telemetry.addData("3.Current ang acc", waddler.getCurrentSituation().getAcceleration().getAngAcc().getValueSI());

            telemetry.addData("4.Target x acc", waddler.getTargetSituation().getAcceleration().getX().getValueSI());
            telemetry.addData("4.Target y acc", waddler.getTargetSituation().getAcceleration().getY().getValueSI());
            telemetry.addData("4.Target ang acc", waddler.getTargetSituation().getAcceleration().getAngAcc().getValueSI());

            telemetry.addData("5.Last Target x vel", waddler.getLastTargetSituation().getVelocity().getX().getValueSI());
            telemetry.addData("5.Last Target y vel", waddler.getLastTargetSituation().getVelocity().getY().getValueSI());
            telemetry.addData("5.Last Target ang vel", waddler.getLastTargetSituation().getVelocity().getAngVel().getValueSI());
        }
    }
}
