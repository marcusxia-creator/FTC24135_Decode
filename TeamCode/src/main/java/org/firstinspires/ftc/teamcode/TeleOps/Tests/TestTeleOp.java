package org.firstinspires.ftc.teamcode.TeleOps.Tests;

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

import org.firstinspires.ftc.teamcode.TeleOps.BallColor;
import org.firstinspires.ftc.teamcode.TeleOps.ColorDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.Turret;


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
    private Turret turret;

    private static double voltage;
    private BallColor ballColor;
    private ColorDetection colorDetection;

    double intakeSpeed = 0.5;
    double shooterPower = 0.0;
    public static double targetShooterRPM = 0.0;
    double currentShooterRPM;
    public static double tickToRPM = 60/28;
    private int targetSlot;

    private PIDController pidController;


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

        robotDrive = new RobotDrive(robot, gamepad_1, gamepad_2);

        turret = new Turret(robot);

        robot.kickerServo.setPosition(RobotActionConfig.kickerOut);
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot0);
        robot.shooterAdjusterServo.setPosition(RobotActionConfig.shooterAdjusterMax);

        colorDetection = new ColorDetection(robot);
        pidController = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);
        tickToRPM = (60/28); // for (tick/s) * 60 (s/min) /28 (tick per rotation)
        targetShooterRPM = 0;

    }

    public void SpindexerRunTo(int slot){
        if (slot==0){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlotShoot);
        }
        if(slot==1){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot0);
        }
        if(slot==2){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);
        }
        if(slot==3){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot2);
        }
    }

    @Override
    public void loop() {

        robot.pinpoint.update();
        voltage = robot.getBatteryVoltageRobust();
        //speed = shooterPowerAngleCalculator.getPower();
        //double power_setpoint = speed*12.0/voltage;
        pidController.setPID(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);

        robotDrive.DriveLoop();
        ballColor = BallColor.fromHue(colorDetection.getHue());
        currentShooterRPM = robot.topShooterMotor.getVelocity() * tickToRPM;
        shooterPower = pidController.calculate(currentShooterRPM, Range.clip(targetShooterRPM,0,6000));
        shooterPower = Math.max(-1.0, Math.min(1.0, shooterPower));
        robot.topShooterMotor.setPower(Range.clip(shooterPower,0.0,1.0));
        robot.bottomShooterMotor.setPower(Range.clip(shooterPower,0.0,1.0));

        /*fine tuning controls - gamepad 1*/
        /** kicker servo fine tune*/
        if (gamepad_1.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            servoposition = robot.kickerServo.getPosition() + 0.01;
            robot.kickerServo.setPosition(Range.clip(servoposition, 0.0, 1.0
            ));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
            servoposition = robot.kickerServo.getPosition() - 0.01;
            robot.kickerServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        /** spindexer servo fine tune*/
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() + 0.01;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() - 0.01;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        /** shooter adjuster servo fine tune */
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && isButtonDebounced()) {
            servoposition = robot.shooterAdjusterServo.getPosition() + 0.05;
            robot.shooterAdjusterServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && isButtonDebounced()) {
            servoposition = robot.shooterAdjusterServo.getPosition() - 0.05;
            robot.shooterAdjusterServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        /** shooter speed up tune*/
        if (gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced()){
            targetShooterRPM += 400;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()){
            targetShooterRPM -= 400;
        }

        /** intake motor run tune*/
        if (gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && isButtonDebounced()) {
            robot.intakeMotor.setPower(Range.clip(intakeSpeed, 0.5, 1.0));
            intakeSpeed += 0.05;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isButtonDebounced()){
            robot.intakeMotor.setPower(0);
            intakeSpeed = 0.0;
        }

        /*action controls - gamepad 2*/
        /**spindexer servo run slots*/
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            targetSlot ++;
            targetSlot = Math.floorMod(targetSlot, 3);
            targetSlot = Range.clip(targetSlot,0,3);
            SpindexerRunTo(targetSlot);
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            targetSlot --;
            targetSlot = Math.floorMod(targetSlot, 3);
            targetSlot = Range.clip(targetSlot,0,3);
            SpindexerRunTo(targetSlot);
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_DOWN) && isButtonDebounced()) {
            SpindexerRunTo(0);
        }

        /**kicker servo up and down*/
        if (gamepad_2.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            robot.kickerServo.setPosition(RobotActionConfig.kickerOut);
        }
        if (gamepad_2.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
            robot.kickerServo.setPosition(RobotActionConfig.kickerIn);
        }

        /**intake start stop*/
        if (gamepad_2.getButton(GamepadKeys.Button.LEFT_BUMPER) && isButtonDebounced()) {
            robot.intakeMotor.setPower(0.95);
        }
        if (gamepad_2.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isButtonDebounced()) {
            robot.intakeMotor.setPower(0);
        }

        /** shooter adjuster trigger */
        if (gamepad_2.getButton(GamepadKeys.Button.X) && isButtonDebounced()) {
            robot.shooterAdjusterServo.setPosition(RobotActionConfig.shooterAdjusterMax);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()) {
            robot.shooterAdjusterServo.setPosition(RobotActionConfig.shooterAdjusterMin);
        }

        /// LED color
        if (ballColor == BallColor.UNKNOWN) {
            robot.LED.setPosition(0.0);
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

        telemetry.addData("Robot Voltage", robot.getBatteryVoltageRobust());
        telemetry.addData("Shooter target RPM", targetShooterRPM);
        telemetry.addData("Shooter current RPM", currentShooterRPM);
        telemetry.addData("Shooter top velocity", robot.topShooterMotor.getVelocity());
        telemetry.addData("Shooter Motor Power Calculator", shooterPower);
        telemetry.addLine("----------------------------------------------------");
        telemetry.addData("Color", ballColor);
        telemetry.addLine("----------------------------------------------------");
        telemetry.addData("turret target angle - atan", turret.getTargetAngle());
        telemetry.addData("turret motor tick", robot.turretMotor.getCurrentPosition());
        telemetry.addData("turret motor angle", turret.getTurretMotorAngle());
        telemetry.addData("turret motor drive angle", turret.getTurretDriveAngle());
        telemetry.addData("turret motor drive tick", turret.motorDriveTick());
        telemetry.addLine("----------------------------------------------------");

        telemetry.addData("pinpoint Pose2D", robot.pinpoint.getPosition());
        telemetry.update();
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
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
