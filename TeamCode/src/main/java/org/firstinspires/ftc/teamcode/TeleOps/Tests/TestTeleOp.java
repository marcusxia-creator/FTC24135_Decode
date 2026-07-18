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

import org.firstinspires.ftc.teamcode.TeleOps.GamepadComboInput;
import org.firstinspires.ftc.teamcode.TeleOps.ShooterPowerCalculator;
import org.firstinspires.ftc.teamcode.TeleOps.Limelight;

import org.firstinspires.ftc.teamcode.TeleOps.RobotDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.SlotSensor;
import org.firstinspires.ftc.teamcode.TeleOps.SpindexerUpd;
import org.firstinspires.ftc.teamcode.TeleOps.Turret;


@Config
@TeleOp (name = "TestTeleOp", group = "org.firstinspires.ftc.teamcode.Tests")
public class TestTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private GamepadComboInput gamepadComboInput;
    /// Subsystem
    private RobotDrive robotDrive;
    private ShooterPowerCalculator powerCalculator;
    private Turret turret;
    private SlotSensor slotSensor;
    private Limelight limelight;
    private SpindexerUpd spindexer;

    ///timer
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime pressedTimer = new ElapsedTime();
    private static final double DEBOUNCE_THRESHOLD = 0.1;

    /// variables
    private double servoposition;
    double shooterPower = 0.0;
    public static double targetShooterRPM = 0.0;
    double currentShooterRPM = 0;
    public static double shooterTickToRPMConversion = (60/28); // for (tick/s) * 60 (s/min) /28 (tick per rotation)

    public SHOOTERMOTORSTATE shootermotorstate;

    public double intakeSpeed = 0.0;

    public double targetTurretPosition = 0;

    /// PID Controller
    private PIDController pidController;
    private PIDController pidController_turret;
    private boolean finetune = false;
    private boolean pidstatus = false;

    /// Status
    private boolean turretStatus = false;
    private boolean resetTurret = false;


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

        robotDrive = new RobotDrive(robot, new GamepadComboInput(gamepad_1, gamepad_2));
        robotDrive.Init();

        spindexer = new SpindexerUpd(robot, SpindexerUpd.SLOT.Empty, SpindexerUpd.SLOT.Empty, SpindexerUpd.SLOT.Empty, 0);
        powerCalculator = new ShooterPowerCalculator(robot);

        turret = new Turret(robot, true);

        limelight = new Limelight(robot);
        limelight.initLimelight(24);
        limelight.start();

        pidController = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);
        pidController_turret = new PIDController(PIDTuning.kPTurret, PIDTuning.kITurret, PIDTuning.kDTurret);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override

    public void loop() {
        /// Robot Kinetics & Localization
        robot.pinpoint.update();
        robotDrive.DriveLoop();

        /// color detection


        ///  PID Controller for power calculation
        pidController.setPID(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);
        currentShooterRPM = robot.topShooterMotor.getVelocity() * shooterTickToRPMConversion;


        /// PID Controller and Shooter Power status
        if (shootermotorstate == SHOOTERMOTORSTATE.RUN && pidstatus){
            shooterPower = pidController.calculate(currentShooterRPM, Range.clip(targetShooterRPM,0,5500));
            robot.topShooterMotor.setPower(Range.clip(shooterPower,0,1));
            robot.bottomShooterMotor.setPower(Range.clip(shooterPower,0,1));
        }
        if (shootermotorstate == SHOOTERMOTORSTATE.RUN && !pidstatus) {
            robot.topShooterMotor.setPower(Range.clip(shooterPower,0,1));
            robot.bottomShooterMotor.setPower(Range.clip(shooterPower,0,1));
        }
        if (shootermotorstate == SHOOTERMOTORSTATE.STOP) {
            robot.topShooterMotor.setPower(0);
            robot.bottomShooterMotor.setPower(0);
        }

        /// PID Controller for Turret
        if (turretStatus){
            pidController_turret.setPID(PIDTuning.kPTurret, PIDTuning.kITurret, PIDTuning.kDTurret);
            double turretPower = pidController_turret.calculate(turret.getTurretMotorAngle(),targetTurretPosition);
            robot.turretMotor.setPower(Range.clip(turretPower, -1, 1));
        }
        if (!turretStatus){
            robot.turretMotor.setPower(0);
        }

        /** run kicker servoposition*/
        if (gamepad_1.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            servoposition = robot.kickerServo.getPosition() + 0.01;
            robot.kickerServo.setPosition(Range.clip(servoposition, 0.0, 1.0
            ));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
            servoposition = robot.kickerServo.getPosition() - 0.01;
            robot.kickerServo.setPosition(Range.clip(servoposition, 0.0, 1.0));
        }
        /** run spindexer servoposition*/
        if (gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && !gamepad_1.getButton(GamepadKeys.Button.Y) && !isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() + 0.01;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() - 0.01;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        /** shooter adjuster */
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && isButtonDebounced()) {
            servoposition = robot.shooterAdjusterServo.getPosition() + 0.01;
            robot.shooterAdjusterServo.setPosition(Range.clip(servoposition, 0, 0.5));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && isButtonDebounced()) {
            servoposition = robot.shooterAdjusterServo.getPosition() - 0.01;
            robot.shooterAdjusterServo.setPosition(Range.clip(servoposition, 0, 0.5));
        }
        /** run shooter target RPM */
        if (gamepad_1.getButton(GamepadKeys.Button.X) && !gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && isButtonDebounced()){
            shootermotorstate = SHOOTERMOTORSTATE.RUN;
            finetune = true;
            pidstatus = true;
            targetShooterRPM += 200;
        }

        if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()){
            finetune = true;
            pidstatus = true;
            targetShooterRPM -= 200;
        }

        if (gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && gamepad_1.getButton(GamepadKeys.Button.Y)&& isButtonDebounced()){
            shootermotorstate = SHOOTERMOTORSTATE.RUN;
            finetune = true;
            pidstatus = false;
            shooterPower = 0;
        }

        /** run intake motor */
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()){
            robot.intakeMotor.setPower(Range.clip(intakeSpeed, 0.5, 1.0));
            intakeSpeed += 0.05;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()){
            robot.intakeMotor.setPower(0);
        }

        /** run turret motor */
        if (gamepad_1.getButton(GamepadKeys.Button.START) && isButtonDebounced()){
            turretStatus = !turretStatus;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON) && isButtonDebounced()){
            targetTurretPosition += 100;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) && isButtonDebounced()){
            targetTurretPosition -= 100;
        }

        /**
         * GamePad#2 to drive the Robot subsystem to position
         */

        /** intake Motor*/
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            robot.intakeMotor.setPower(0.75);
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            robot.intakeMotor.setPower(0);
        }

        /** run kicker servoposition*/
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
        if (gamepad_2.getButton(GamepadKeys.Button.LEFT_BUMPER) && isButtonDebounced()) {
            spindexer.RunToNext();
        }
        if (gamepad_2.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isButtonDebounced()) {
            spindexer.RuntoPosition(0);
        }

        /** run shooter based on target distance*/
        if (gamepad_2.getButton(GamepadKeys.Button.X) && isButtonDebounced()) {
            finetune = false;
            pidstatus = true;
            targetShooterRPM = powerCalculator.getRPM();
        }
        if (gamepad_2.getButton(GamepadKeys.Button.Y) && isButtonDebounced()) {
            finetune = false;
            pidstatus = false;
            targetShooterRPM = 0;
        }

        /** run turret*/
        if (gamepad_2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON) && isButtonDebounced()) {
            turretStatus = true;
            targetTurretPosition = turret.getTargetAngle();
        }
        if (gamepad_2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) && isButtonDebounced()) {
            turretStatus = false;
        }


        /**
         * LED alarm light
         */
        if (powerCalculator.getDistance() <= 54) {
            robot.LED.setPosition(0.28);
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
        telemetry.addData("shooter RPM", robot.topShooterMotor.getVelocity() * shooterTickToRPMConversion);
        telemetry.addLine("----------------------------------------------------");

        telemetry.addLine("-----------Turret-----------------------------------");
        telemetry.addData("turret tune Status", turretStatus);
        telemetry.addData("turret motor tick", robot.turretMotor.getCurrentPosition());
        telemetry.addData("turret motor angle", turret.getTurretMotorAngle());
        telemetry.addData("turret motor drive angle", turret.getTurretDriveAngle());

        telemetry.update();
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
    private boolean isLimitSwitchPressed(){
        boolean switchState = robot.limitSwitch.getState();
        if (switchState){
            if (pressedTimer.seconds() >= 0.2){
                return true;
            }
        } else{
            pressedTimer.reset();
        }
        return false;
    }

    @Config
    public static class PIDTuning {
        public static double kP = 0.001;
        public static double kI = 0.000001;
        public static double kD = 0.00001;
        public static double kPTurret = 0.00001; // position or RPM target
        public static double kITurret = 0.00001; // position or RPM target
        public static double kDTurret = 0.00001; // position or RPM target
    }
}