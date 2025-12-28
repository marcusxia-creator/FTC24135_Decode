package org.firstinspires.ftc.teamcode.IceWaddler.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
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
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import java.util.ArrayList;

@TeleOp(name="Vel PID Tuner", group="Icewaddler Tests")

public class VelPIDTuner extends OpMode {
    public RobotHardware robot;
    public IceWaddler iceWaddler;
    public FtcDashboard dashboard;

    //Config Vars
    public static double writeInterval = 0.1;
    public static Pose2D initPose = new Pose2D(METER,0,0, AngleUnit.DEGREES,-90);

    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        iceWaddler = new IceWaddler(robot);
        iceWaddler.Init(IceWaddler.CONTROLMODE.POWER, initPose, true);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("Current Pose", initPose);
        telemetry.addData("Write Interval:", writeInterval);
        telemetry.update();
    }

    public void loop(){
        iceWaddler.runByVel(new Pose2D(METER, 2*gamepad1.right_stick_x, 2*gamepad1.right_stick_y, DEGREES,0), 30*gamepad1.left_stick_x);
        iceWaddler.loop();
        telemetry.addData("Current Pose",iceWaddler.currentPos);

        telemetry.addData("xVel",iceWaddler.currentVel.getX(METER));
        telemetry.addData("TxVel",iceWaddler.targetVel.getX(METER));
        telemetry.addData("xPow",iceWaddler.targetPower.getX(METER));
        telemetry.addData("yVel",iceWaddler.currentVel.getY(METER));
        telemetry.addData("TyVel",iceWaddler.targetVel.getY(METER));
        telemetry.addData("yPow",iceWaddler.targetPower.getY(METER));
        telemetry.addData("rVel",iceWaddler.currentRotVel);
        telemetry.addData("TrVel",iceWaddler.targetRotVel);
        telemetry.addData("TrPow",iceWaddler.targetRotPower);

        double x=iceWaddler.currentPos.getX(INCH);
        double y=iceWaddler.currentPos.getY(INCH);
        double h=-iceWaddler.currentPos.getHeading(RADIANS);
        TelemetryPacket packet=new TelemetryPacket(true);
        packet.fieldOverlay()
                .setAlpha(0.25)
                .setFill("white")
                .fillPolygon(   new double[]{x, x+9*sin(h), x+sqrt(162)*sin(h+PI/4),    x+sqrt(162)*sin(h+3*PI/4),  x+sqrt(162)*sin(h-3*PI/4),  x+sqrt(162)*sin(h-PI/4),    x+9*sin(h)},
                                new double[]{y, y+9*cos(h), y+sqrt(162)*cos(h+PI/4),    y+sqrt(162)*cos(h+3*PI/4),  y+sqrt(162)*cos(h-3*PI/4),  y+sqrt(162)*cos(h-PI/4),    y+9*cos(h)})
                .setAlpha(1)
                .setStroke("white")
                .setStrokeWidth(2)
                .strokePolygon( new double[]{x, x+9*sin(h), x+sqrt(162)*sin(h+PI/4),    x+sqrt(162)*sin(h+3*PI/4),  x+sqrt(162)*sin(h-3*PI/4),  x+sqrt(162)*sin(h-PI/4),    x+9*sin(h)},
                                new double[]{y, y+9*cos(h), y+sqrt(162)*cos(h+PI/4),    y+sqrt(162)*cos(h+3*PI/4),  y+sqrt(162)*cos(h-3*PI/4),  y+sqrt(162)*cos(h-PI/4),    y+9*cos(h)});
        dashboard.sendTelemetryPacket(packet);
    }
}
