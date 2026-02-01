package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOps.LUTPowerCalculator;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;

import org.firstinspires.ftc.teamcode.TeleOps.RobotDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.Turret;
import org.firstinspires.ftc.teamcode.TeleOps.LimelightTest;

@Config
@TeleOp (name = "TestTeleOp", group = "org.firstinspires.ftc.teamcode")
public class TestTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private double servoposition;
    private static double speed;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private RobotDrive robotDrive;
    private LUTPowerCalculator powerCalculator;
    private Turret turret;

    private BallColor ballColor;
    private ColorDetection colorDetection;

    double intakeSpeed = 0.5;
    double shooterPower = 0.0;
    public static double targetShooterRPM = 0.0;
    double currentShooterRPM = 0;
    public static double tickToRPM;

    private LimelightTest limelightTest;

    private PIDController pidController;
    private LUTPowerCalculator shooterPowerLUT;

    private boolean finetune = false;
    private boolean pidstatus = false;
    private boolean turretStatus = false;

    public static double adjusterServoPosition = 0.49;


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

        turret = new Turret(robot);

        limelightTest = new LimelightTest(robot, turret);
        limelightTest.initLimelight(24);
        limelightTest.start();

        colorDetection = new ColorDetection(robot);
        pidController = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);
        tickToRPM = (60/28); // for (tick/s) * 60 (s/min) /28 (tick per rotation)
    }

    @Override
    public void loop() {
        /// Robot pinpoint
        robot.pinpoint.update();
        robotDrive.DriveLoop();

        /// color detection
        ballColor = BallColor.fromHue(colorDetection.getHue());

        ///  PID Controller for power calculation
        pidController.setPID(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);
        currentShooterRPM = robot.topShooterMotor.getVelocity() * tickToRPM;
        //targetShooterRPM = shooterPowerLUT.getPower();

        robot.shooterAdjusterServo.setPosition(adjusterServoPosition);

        /// PID Controller and power status
        /*
        if (finetune & pidstatus) {
            shooterPower = pidController.calculate(currentShooterRPM, Range.clip(targetShooterRPM,0,6000));
        }
        else if (!finetune & pidstatus){
            shooterPower = shooterPowerLUT.getPower();
        }
        else{
            shooterPower = 0;
        }
         */


        shooterPower = pidController.calculate(currentShooterRPM, Range.clip(targetShooterRPM,0,6000));
        //shooterPower = 1;
        robot.topShooterMotor.setPower(shooterPower);

        /**
        /// run turret
        if (turretStatus){
            turret.driveTurretMotor();
        }
        else{
            robot.turretMotor.setPower(0);
        }

        if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()) {
            robot.kickerServo.setPosition(kickerRetract);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced()) {
            robot.kickerServo.setPosition(kickerExtend);
        }

        if (gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && isButtonDebounced()) {
            robot.spindexerServo.setPosition(spindexerSlot3);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isButtonDebounced()) {
            robot.spindexerServo.setPosition(0);
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            robot.spindexerServo.setPosition(spindexerSlot1);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            robot.spindexerServo.setPosition(spindexerSlot2);
        }

        if (gamepad_1.getButton(GamepadKeys.Button.A)) {
            robot.intakeMotor.setPower(0.9);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.B)) {
            robot.intakeMotor.setPower(0);
        }

        /**
        shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
        ///  set shooter power
        robot.topShooterMotor.setPower(Range.clip(shooterPower,0.0,1.0));
        robot.bottomShooterMotor.setPower(Range.clip(shooterPower,0.0,1.0));
         */


        /** run kicker servoposition*/
        /**
        if (gamepad_1.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            servoposition = robot.kickerServo.getPosition() + 0.05;
            robot.kickerServo.setPosition(Range.clip(servoposition, 0.0, 1.0
            ));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
            servoposition = robot.kickerServo.getPosition() - 0.05;
            robot.kickerServo.setPosition(Range.clip(servoposition, 0.0, 1.0));
        }
        /** run spindexer servoposition*/
        /**
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() + 0.01;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() - 0.01;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        /** shooter adjuster */
        /**
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && isButtonDebounced()) {
            servoposition = robot.shooterAdjusterServo.getPosition() + 0.01;
            robot.shooterAdjusterServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && isButtonDebounced()) {
            servoposition = robot.shooterAdjusterServo.getPosition() - 0.01;
            robot.shooterAdjusterServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        /** run shooter target RPM */
        /**
        if (gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced()){
            finetune = true;
            pidstatus = true;
            targetShooterRPM += 200;
        }

        if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()){
            finetune = true;
            pidstatus = true;
            targetShooterRPM -= 200;
        }

        /** run intake motor*/
        /**
        if (gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && isButtonDebounced()) {
            robot.intakeMotor.setPower(Range.clip(intakeSpeed, 0.5, 1.0));
            intakeSpeed += 0.05;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isButtonDebounced()){
            robot.intakeMotor.setPower(0);
        }

        /**
         * GamePad#2 to drive the spindexer
         */
        /** run kicker servoposition*/
        /**
        if (gamepad_2.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            servoposition = kickerRetract;
            robot.kickerServo.setPosition(Range.clip(servoposition, 0.0, 1.0
            ));
        }
        if (gamepad_2.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
            servoposition = kickerExtend;
            robot.kickerServo.setPosition(Range.clip(servoposition, 0.0, 1.0));
        }
        /** run spindexer per slot*/
        /**
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() + slotAngleDelta;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() - slotAngleDelta;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        /** run shooter based on target distance*/
        /**
        if (gamepad_2.getButton(GamepadKeys.Button.X) && isButtonDebounced()) {
            finetune = false;
            pidstatus = true;
        }
        if (gamepad_2.getButton(GamepadKeys.Button.Y) && isButtonDebounced()) {
            finetune = false;
            pidstatus = false;
        }

        /** run turret*/
        /**
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_UP) && isButtonDebounced()) {
            turretStatus = true;
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_DOWN) && isButtonDebounced()) {
            turretStatus = false;
        }


        /**
         * LED alarm light
         */
        if (powerCalculator.getDistance() <= 54) {
            robot.LED.setPosition(0.28);
        }
        else if (ballColor.isKnown()) {
            if (ballColor == BallColor.GREEN) {
                robot.LED.setPosition(0.5);
            }
            if (ballColor == BallColor.PURPLE) {
                robot.LED.setPosition(0.722);
            }
        }
        else {
            robot.LED.setPosition(1.0);
        }


        telemetry.addData("Kicker Postion", robot.kickerServo.getPosition());
        telemetry.addData("Spindexer Position", robot.spindexerServo.getPosition());
        telemetry.addData("Shooter Adjuster Postion", robot.shooterAdjusterServo.getPosition());
        telemetry.addData("Shooter Acutal Power", robot.topShooterMotor.getPower());
        telemetry.addData("Intake Speed", robot.intakeMotor.getPower());
        telemetry.addLine("----------------------------------------------------");
        telemetry.addData("Pose 2D", robot.pinpoint.getPosition());
        telemetry.addData("Distance To Goal", powerCalculator.getDistance());
        telemetry.addData("Shooter power now", shooterPower);
        telemetry.addData("shooter velocity", robot.topShooterMotor.getVelocity());
        telemetry.addData("shooter RPM", robot.topShooterMotor.getVelocity() * tickToRPM);
        telemetry.addLine("----------------------------------------------------");
        telemetry.addData("Color", ballColor);
        telemetry.addLine("----------------------------------------------------");
        telemetry.addData("turret target angle - atan", turret.getTargetAngle());
        telemetry.addData("turret motor tick", robot.turretMotor.getCurrentPosition());
        telemetry.addData("turret motor angle", turret.getTurretMotorAngle());
        telemetry.addData("turret motor drive angle", turret.getTurretDriveAngle());
        telemetry.addData("turret motor drive tick", turret.motorDriveTick());
        telemetry.addLine("----------------------------------------------------");
        Pose2D MT2Pose = limelightTest.updateTagMT2(DistanceUnit.MM);
        Pose2D MT2Offset = limelightTest.updateTagMT2OFFSET(DistanceUnit.MM);
        Pose2D turretOffset = limelightTest.updateTagMT2OFFSET2(DistanceUnit.MM);
        Pose2D MT2Normalize = limelightTest.updateTagMT2NORMALIZED(DistanceUnit.MM);
        if (MT2Pose != null && MT2Offset != null && MT2Normalize != null) {
            telemetry.addData("limelight Pose2D", "%.2f, %.2f, %.2f", MT2Pose.getX(DistanceUnit.MM), MT2Pose.getY(DistanceUnit.MM), MT2Pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("limelight offset", "%.2f, %.2f, %.2f", MT2Offset.getX(DistanceUnit.MM), MT2Offset.getY(DistanceUnit.MM), MT2Offset.getHeading(AngleUnit.DEGREES));
            telemetry.addData("turret offset", "%.2f, %.2f, %.2f", turretOffset.getX(DistanceUnit.MM), turretOffset.getY(DistanceUnit.MM), turretOffset.getHeading(AngleUnit.DEGREES));
            telemetry.addData("limelight normalized", "%.2f, %.2f, %.2f", MT2Normalize.getX(DistanceUnit.MM), MT2Normalize.getY(DistanceUnit.MM), MT2Normalize.getHeading(AngleUnit.DEGREES));
        }

        telemetry.addData("pinpoint Pose2D", robot.pinpoint.getPosition());
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