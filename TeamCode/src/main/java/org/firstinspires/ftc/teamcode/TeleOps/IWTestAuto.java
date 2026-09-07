package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.maxSpeed;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position.ORIGIN;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.ArcDirection;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.HeadingProfile;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements.*;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@Autonomous(name="IceWaddler Test Auto")
public class IWTestAuto extends ScheduledOpMode {
    RobotHardware robot;
    IceWaddler waddler;

    Action path;

    @Override
    public void init() {
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(ORIGIN,false);

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        path=new ActionSeries(
                waddler.new InitPath(),
                waddler.new MotionAction(new Line(new PathingPoint(new Position(new Vector(-1,0,m),new NormalizedAngle(-90,deg)),new Scalar(0.6,metersPerSecond)),new maxSpeedMP(maxSpeed),new SCurveHP(),new String[]{})),
                waddler.new MotionAction(new FixedArc(new Vector(-1,0.5,m), new PositiveAngle(90,deg), ArcDirection.CLOCKWISE,new Scalar(0,metersPerSecond),new NormalizedAngle(-90,deg),new maxSpeedMP(new Scalar(0.6,metersPerSecond)),new SCurveHP(),new String[]{})),
                new ActionParallel(ActionParallel.TERMINATIONTYPE.ALL,waddler.new MotionAction(new holdPos(new String[]{})),new Delay(3)),
                waddler.new MotionAction(new Line(new PathingPoint(new Position(new Vector(-0.5,0.5,m),new NormalizedAngle(-90,deg)),new Scalar(0.3,metersPerSecond)),new maxSpeedMP(maxSpeed),new LUT_HP(new double[]{0.5}, new NormalizedAngle[]{new NormalizedAngle(180,deg)},new HeadingProfile[]{new SCurveHP()}),new String[]{})),
                waddler.new MotionAction(new FixedArc(new Vector(-0.5,0.25,m),new PositiveAngle(90,deg),ArcDirection.CLOCKWISE,new Scalar(0.3,metersPerSecond),new NormalizedAngle(-45,deg),new linearMP(),new linearHP(),new String[]{})),
                waddler.new MotionAction(new FixedArc(new Vector(0,0.25,m),new PositiveAngle(90,deg),ArcDirection.COUNTERCLOCKWISE,new Scalar(0,metersPerSecond),new NormalizedAngle(0,deg),new maxSpeedMP(new Scalar(0.3,metersPerSecond)),new linearHP(),new String[]{})),
                new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,waddler.new MotionAction(new holdPos(new String[]{})),new Standby())
        );

        rootAction=new ActionParallel(ActionParallel.TERMINATIONTYPE.NONE,
                path,
                new telemetryDriver()
        );
    }

    class telemetryDriver implements Action{
        public telemetryDriver(){}

        @Override
        public void loop() {
            waddler.drawField();
            telemetry.addData("Ticktime",waddler.getTickTime().getValueSI());

            telemetry.addData("1a.Current x pos", waddler.getCurrentSituation().getPosition().getX().getValueSI());
            telemetry.addData("1a.Current y pos", waddler.getCurrentSituation().getPosition().getY().getValueSI());
            telemetry.addData("1a.Current heading", waddler.getCurrentSituation().getPosition().getHeading().getValueSI());

            if(waddler.getTargetSituation().getPosition()!=null) {
                telemetry.addData("1b.Target x pos", waddler.getTargetSituation().getPosition().getX().getValueSI());
                telemetry.addData("1b.Target y pos", waddler.getTargetSituation().getPosition().getY().getValueSI());
                telemetry.addData("1b.Target heading", waddler.getTargetSituation().getPosition().getHeading().getValueSI());
            }

            telemetry.addData("2a.Current x vel", waddler.getCurrentSituation().getVelocity().getX().getValueSI());
            telemetry.addData("2a.Current y vel", waddler.getCurrentSituation().getVelocity().getY().getValueSI());
            telemetry.addData("2a.Current ang vel", waddler.getCurrentSituation().getVelocity().getAngVel().getValueSI());

            if(waddler.getTargetSituation().getVelocity()!=null) {
                telemetry.addData("2b.Target x vel", waddler.getTargetSituation().getVelocity().getX().getValueSI());
                telemetry.addData("2b.Target y vel", waddler.getTargetSituation().getVelocity().getY().getValueSI());
                telemetry.addData("2b.Target ang vel", waddler.getTargetSituation().getVelocity().getAngVel().getValueSI());
            }

            telemetry.addData("3a.Current x acc", waddler.getCurrentSituation().getAcceleration().getX().getValueSI());
            telemetry.addData("3a.Current y acc", waddler.getCurrentSituation().getAcceleration().getY().getValueSI());
            telemetry.addData("3a.Current ang acc", waddler.getCurrentSituation().getAcceleration().getAngAcc().getValueSI());

            if(waddler.getLastTargetSituation().getAcceleration()!=null) {
                telemetry.addData("3b.Target x acc", waddler.getTargetSituation().getAcceleration().getX().getValueSI());
                telemetry.addData("3b.Target y acc", waddler.getTargetSituation().getAcceleration().getY().getValueSI());
                telemetry.addData("3b.Target ang acc", waddler.getTargetSituation().getAcceleration().getAngAcc().getValueSI());
            }

            if(waddler.getCurrentAction()!=null) {
                telemetry.addData("4.Completion",waddler.getCurrentAction().getCompletion());
            }

            //telemetry.addData("4.Target Pathingpoint exists",waddler.targetPathingPoint!=null);

            telemetry.update();
        }
    }
}
