package org.firstinspires.ftc.teamcode.IceWaddler.tests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler.CONTROLMODE;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import java.util.ArrayList;

@TeleOp(name="Odo Tracker", group="Icewaddler Tests")
@Config
public class odoTest extends OpMode {
    public RobotHardware robot;
    public IceWaddler iceWaddler;
    public ArrayList<Pose2D> poses;
    public ElapsedTime writeTime;
    public FtcDashboard dashboard;

    //Config Vars
    public static double writeInterval = 0.1;
    public static Pose2D initPose = new Pose2D(METER,0,0,AngleUnit.DEGREES,0);

    public void init(){
        robot=new RobotHardware(hardwareMap);
        robot.init(hardwareMap);
        robot.initIMU();

        iceWaddler=new IceWaddler(robot);
        iceWaddler.Init(CONTROLMODE.POWER,initPose,true);

        dashboard=FtcDashboard.getInstance();
        telemetry=new MultipleTelemetry(telemetry,dashboard.getTelemetry());
        telemetry.addData("Current Pose",initPose);
        telemetry.addData("Write Interval:",writeInterval);
        telemetry.update();
    }

    public void start(){
        writeTime.reset();
        poses.add(initPose);
    }

    public void loop(){
        iceWaddler.loop();
        telemetry.addData("Current Pose",iceWaddler.currentPos);

        double x=iceWaddler.currentPos.getX(INCH);
        double y=iceWaddler.currentPos.getY(INCH);
        double h=iceWaddler.currentPos.getHeading(RADIANS);
        TelemetryPacket packet=new TelemetryPacket(true);
        packet.fieldOverlay()
                .setAlpha(0.25)
                .setFill("white")
                .fillPolygon(   new double[]{x, x+9*sin(h), x+sqrt(162)*sin(h+PI/4),    x+sqrt(162)*sin(h+3*PI/4),  x+sqrt(162)*sin(h-3*PI/4),  x+sqrt(162)*sin(h-PI/4),    x+9*sin(h)},
                                new double[]{y, y+9*cos(h), y+sqrt(162)*cos(h+PI/4),    y+sqrt(162)*cos(h+3*PI/4),  y+sqrt(162)*cos(h-3*PI/4),  y+sqrt(162)*cos(h-PI/4),    y+9*cos(h)})
                .setAlpha(1)
                .setStroke("white")
                .setStrokeWidth(3)
                .strokePolygon( new double[]{x, x+9*sin(h), x+sqrt(162)*sin(h+PI/4),    x+sqrt(162)*sin(h+3*PI/4),  x+sqrt(162)*sin(h-3*PI/4),  x+sqrt(162)*sin(h-PI/4),    x+9*sin(h)},
                                new double[]{y, y+9*cos(h), y+sqrt(162)*cos(h+PI/4),    y+sqrt(162)*cos(h+3*PI/4),  y+sqrt(162)*cos(h-3*PI/4),  y+sqrt(162)*cos(h-PI/4),    y+9*cos(h)})
                .setStroke("green")
                .setStrokeWidth(1);
        for(int i=1;i>=poses.toArray().length;i++){
            packet.fieldOverlay().strokeLine(poses.get(i-1).getX(INCH),poses.get(i-1).getY(INCH),
                    poses.get(i).getX(INCH),poses.get(i).getY(INCH));
        }
        dashboard.sendTelemetryPacket(packet);

        if(writeTime.seconds()>writeInterval){
            writeTime.reset();
            poses.add(iceWaddler.currentPos);
        }
    }
}
