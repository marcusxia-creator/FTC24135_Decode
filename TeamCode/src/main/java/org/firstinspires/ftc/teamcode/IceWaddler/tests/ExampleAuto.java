package org.firstinspires.ftc.teamcode.IceWaddler.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler.Action.ACTIONTYPE;
import org.firstinspires.ftc.teamcode.IceWaddler.tests.Paths.ExamplePath;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@TeleOp(name="Example Auto", group="Icewaddler Tests")
public class ExampleAuto extends OpMode {
    public RobotHardware robot;
    public IceWaddler iceWaddler;
    public FtcDashboard dashboard;

    public void init(){
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        iceWaddler=new IceWaddler(robot);
        iceWaddler.Init(IceWaddler.CONTROLMODE.POWER,ExamplePath.init,true);

        dashboard= FtcDashboard.getInstance();
        telemetry=new MultipleTelemetry(telemetry,dashboard.getTelemetry());
    }

    public void start(){
        iceWaddler.runPath(ExamplePath.path);
    }

    public void loop(){
        iceWaddler.loop();
        telemetry.addData("Current Pose", iceWaddler.currentPos);
        telemetry.addData("Current Vel", iceWaddler.currentVel);
        telemetry.addData("Current rVel", iceWaddler.currentRotVel);

        TelemetryPacket packet = new TelemetryPacket();
        double x=iceWaddler.currentPos.getX(INCH);
        double y=iceWaddler.currentPos.getY(INCH);
        double h=-iceWaddler.currentPos.getHeading(RADIANS);
        packet.fieldOverlay()
                .setAlpha(0.25)
                .setFill("white")
                .fillPolygon(   new double[]{x, x+9*sin(h), x+sqrt(162)*sin(h+PI/4),    x+sqrt(162)*sin(h+3*PI/4),  x+sqrt(162)*sin(h-3*PI/4),  x+sqrt(162)*sin(h-PI/4),    x+9*sin(h)},
                        new double[]{y, y+9*cos(h), y+sqrt(162)*cos(h+PI/4),    y+sqrt(162)*cos(h+3*PI/4),  y+sqrt(162)*cos(h-3*PI/4),  y+sqrt(162)*cos(h-PI/4),    y+9*cos(h)})
                .setAlpha(1)
                .setStroke("white")
                .setStrokeWidth(2)
                .strokePolygon( new double[]{x, x+9*sin(h), x+sqrt(162)*sin(h+PI/4),    x+sqrt(162)*sin(h+3*PI/4),  x+sqrt(162)*sin(h-3*PI/4),  x+sqrt(162)*sin(h-PI/4),    x+9*sin(h)},
                        new double[]{y, y+9*cos(h), y+sqrt(162)*cos(h+PI/4),    y+sqrt(162)*cos(h+3*PI/4),  y+sqrt(162)*cos(h-3*PI/4),  y+sqrt(162)*cos(h-PI/4),    y+9*cos(h)})
                .setStroke("green")
                .setStrokeWidth(1);

        if(iceWaddler.currentAction.actionType == ACTIONTYPE.RUN){
            telemetry.addData("Starting Pose", iceWaddler.startingPos);
            telemetry.addData("Target Pose", iceWaddler.targetPos);
            packet.fieldOverlay().strokeLine(iceWaddler.startingPos.getX(INCH), iceWaddler.startingPos.getY(INCH), iceWaddler.targetPos.getX(INCH), iceWaddler.targetPos.getY(INCH));
            telemetry.addData("lat Correction", iceWaddler.latCorrection);
            telemetry.addData("rot Correction", iceWaddler.rotCorrection);
            telemetry.addData("Distance Remaining", iceWaddler.distanceRemaining);
            telemetry.addData("Completion", iceWaddler.actionCompletion);
        }

        if(iceWaddler.currentAction.actionType == ACTIONTYPE.HOLD){
            telemetry.addData("Target Pose", iceWaddler.targetPos);
            telemetry.addData("Target Vel", iceWaddler.targetVel);
            telemetry.addData("Time Remaining", iceWaddler.currentAction.delayLength-iceWaddler.delayTimer.seconds());
        }
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
        iceWaddler.loop();
    }
}
