package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.DEBOUNCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.kickerExtend;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.kickerRetract;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.slotAngleDelta;

import static java.lang.Math.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOps.LUTPowerCalculator;
import org.firstinspires.ftc.teamcode.TeleOps.Limelight;
import org.firstinspires.ftc.teamcode.TeleOps.RobotDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.BallColor;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;
import org.firstinspires.ftc.teamcode.TeleOps.Turret;
import org.firstinspires.ftc.teamcode.TeleOps.TurretUpd;

@Config
@TeleOp (name = "turretTestTeleop", group = "org.firstinspires.ftc.teamcode")
public class TurretTestTeleOp extends OpMode {
    private RobotHardware robot;
    private Turret turret;
    private GamepadEx gamepad_1;
    private ElapsedTime debounceTimer = new ElapsedTime();

    private LimelightTest limelightTest;
    private Limelight limelight;

    enum TurretMode{
        MANUAL,
        PID,
        MOTIONPROFILE,
        LIMELIGHT//Not implemented yet
    }

    public TurretMode mode;

    public static double adjusterServoPosition = 0.49;
    public static double targetAngle = 0;
    public double robotAngle = 0;
    public double turretAngle = 0;
    public int currentTick = 0;
    public int targetTick = 0;
    public int lastTargetTick = 0;

    private final double tickToAngle = ((0.16867469879518 * 360) / 145.1);
    private final double angleToTick = 1.0 / tickToAngle;

    private long lastLoopTime = 0;
    private double loopHz = 0.0;
    private double loopTime = 0.0;

    /// Power status
    public enum SHOOTERMOTORSTATE{
        RUN,
        STOP
    }
    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(robot, true);

        limelightTest = new LimelightTest(robot, turret);
        limelightTest.initLimelight(24);
        limelightTest.start();
        limelight = new Limelight(robot);
        limelight.initLimelight(24);
        limelight.start();

        robot.shooterAdjusterServo.setPosition(adjusterServoPosition);
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mode=TurretMode.MANUAL;

        writeTelemetry();
    }

    @Override

    public void loop() {
        /// Robot pinpoint
        robot.pinpoint.update();

        //Controls
        if(gamepad_1.getButton(B)){
            mode=TurretMode.MANUAL;
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(gamepad_1.getButton(A)){
            mode=TurretMode.PID;
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(gamepad_1.getButton(X)){
            mode=TurretMode.MOTIONPROFILE;
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //Limelight not implemented

        if(mode!=TurretMode.MANUAL) {
            targetAngle+=(int)round(gamepad_1.getLeftX()*3);
        }
        targetAngle=normalize(targetAngle);
        robotAngle=normalize(robot.pinpoint.getHeading(AngleUnit.DEGREES));
        turretAngle=normalize(targetAngle-robotAngle);

        currentTick = turret.getCurrentTick();
        lastTargetTick=targetTick;
        targetTick = (int)Math.round(Range.clip(turretAngle, -175, 175) * angleToTick);

        switch(mode){
            case MANUAL:
                robot.turretMotor.setPower(gamepad_1.getLeftX());
            case PID:
                turret.driveTurretPID(currentTick,targetTick,loopTime);
                turret.updatePidFromDashboard();
                break;
            case MOTIONPROFILE:
                turret.driveTurretMP(currentTick,targetTick,loopTime);
                break;
                //Limelight not implemented
        }
        updateLoopFrequency();
        writeTelemetry();
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    void writeTelemetry(){
        telemetry.addData("1. Mode",mode.name());
        telemetry.addData("1. Target Angle",targetAngle);
        telemetry.addData("1. Robot Angle",robotAngle);
        telemetry.addData("1. Turret Angle",turretAngle);
        telemetry.addData("2. Current Tick",currentTick);
        telemetry.addData("2. Target Tick",targetTick);
        telemetry.addData("2. Target Tick",targetTick);
        telemetry.addData("3. Error",currentTick-targetTick);
        telemetry.addData("TargetTickVel",(targetTick-lastTargetTick)/loopTime);
        telemetry.addData("LoopFreq",loopHz);
        telemetry.addData("LoopTime",loopTime);
        telemetry.update();
    }

    private double floorMod(double x, double y){
        return x-(Math.floor(x/y) * y);
    }

    private double normalize(double angle){
        return floorMod(angle+180,360)-180;
    }

    private void updateLoopFrequency() {

        long now = System.currentTimeMillis();

        if (lastLoopTime != 0) {
            long dtMs = now - lastLoopTime;
            loopTime=(double)dtMs/1000;
            if (dtMs > 0) {
                loopHz = 1000.0 / dtMs;
            }
        }
        lastLoopTime = now;
    }
}