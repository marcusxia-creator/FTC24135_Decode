package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.DEBOUNCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.kickerExtend;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.kickerRetract;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.slotAngleDelta;

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

@Config
@TeleOp (name = "TestTeleOp", group = "org.firstinspires.ftc.teamcode")
public class TurretTestTeleOp extends OpMode {
    private RobotHardware robot;
    private Turret turret;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private double servoposition;
    private static double speed;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private RobotDrive robotDrive;
    private LUTPowerCalculator powerCalculator;

    private BallColor ballColor;
    private ColorDetection colorDetection;

    double intakeSpeed = 0.5;
    double shooterPower = 0.0;
    double power = 0.0;
    public static double targetShooterRPM = 0.0;
    double currentShooterRPM = 0;
    public static double tickToRPM = (60/28); // for (tick/s) * 60 (s/min) /28 (tick per rotation)
    public SHOOTERMOTORSTATE shootermotorstate;
    public double adjusterservoposition;


    private LimelightTest limelightTest;
    private Limelight limelight;

    private PIDController pidController;
    private LUTPowerCalculator shooterPowerLUT;
    //private AprilTagDetection aprilTagDetection;

    private boolean finetune = false;
    private boolean pidstatus = false;
    private boolean turretStatus = false;

    public static double adjusterServoPosition = 0.49;
    int targetTick = 0;

    /// Power status
    public enum SHOOTERMOTORSTATE{
        RUN,
        STOP
    }
    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        robotDrive = new RobotDrive(robot, gamepad_1, gamepad_2);
        robotDrive.Init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        powerCalculator = new LUTPowerCalculator(robot);
        shooterPowerLUT = new LUTPowerCalculator(robot);
        robotDrive = new RobotDrive(robot, gamepad_1, gamepad_2);
        //aprilTagDetection = new AprilTagDetection(robot);

        turret = new Turret(robot, true);

        limelightTest = new LimelightTest(robot, turret);
        limelightTest.initLimelight(24);
        limelightTest.start();
        limelight = new Limelight(robot);
        limelight.initLimelight(24);
        limelight.start();

        colorDetection = new ColorDetection(robot);
        pidController = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);

        robot.shooterAdjusterServo.setPosition(adjusterServoPosition);

        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override

    public void loop() {

        /// Robot pinpoint
        robot.pinpoint.update();
        robotDrive.DriveLoop();

        if (gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && isButtonDebounced()) {
            targetTick += 50;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isButtonDebounced()) {
            targetTick -= 50;
        }


        robot.turretMotor.setTargetPosition(Range.clip(targetTick, -400, 400));
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(0.9);

        telemetry.addData("pinpoint Pose2D", robot.pinpoint.getPosition());
        telemetry.addData("turret position", robot.turretMotor.getCurrentPosition());
        telemetry.update();
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    @Config
    public static class PIDTuning {
        public static double kP = 0.001;
        public static double kI = 0.000001;
        public static double kD = 0.00001; // position or RPM target
    }
}