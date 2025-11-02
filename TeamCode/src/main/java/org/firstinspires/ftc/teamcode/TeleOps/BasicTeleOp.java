package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "Basic TeleOp", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private FSMShooter shooterManualControl;
    private Intake intake;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private SpindexerManualControl spindexerManualControl;

    private GamepadManager gamepadManager;
    private Spindexer spindexer;


    @Override
    public void init() {
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);
        robot.initIMU();
        robot.initOdo();

        gamepadManager= new GamepadManager(gamepad1,gamepad2);
        spindexer = new Spindexer(robot, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, 0); //Change inits for comp
        spindexer.runToSlot(0);
        spindexerManualControl = new SpindexerManualControl(robot, spindexer, gamepadManager);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        shooterManualControl = new FSMShooter(gamepadCo1, gamepadCo2, robot, spindexer, gamepadManager);
        shooterManualControl.Init();

        intake = new Intake(gamepadCo1, gamepadCo2, robot, spindexer, gamepadManager);

    }

    @Override
    public void loop() {
        //robotDrive.DriveLoop();
        gamepadManager.loop();

        shooterManualControl.ShooterLoop();
        intake.loop();

        spindexer.runToSlot();
        spindexerManualControl.loop();

        robotDrive.DriveLoop();

        telemetry.addData("Intake State", intake.intakeStates);
        telemetry.addData("Sensor Distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addData("Current Slot", spindexer.currentSlot);
        telemetry.addData("Spindexer Servo Pos", robot.spindexerServo.getPosition());
        telemetry.addData("Target Colour", shooterManualControl.targetColour.name());
        telemetry.addLine("-----");
        telemetry.addData("Shooter State", shooterManualControl.shooterState);
        telemetry.addData("Shooter Vel", robot.shooterMotor.getVelocity());
        telemetry.addData("Shooter Motor Mode", robot.shooterMotor.getMode());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }
    public boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
}
