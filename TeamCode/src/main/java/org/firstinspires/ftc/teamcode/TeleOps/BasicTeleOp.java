package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Basic TeleOp", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private FSMShooter shooterManualControl;
    private Intake intake;
    private ElapsedTime debounceTimer = new ElapsedTime();

    private GamepadManager gamepadManager;
    private Spindexer spindexer;


    @Override
    public void init() {
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadManager= new GamepadManager(gamepad1,gamepad2);
        spindexer = new Spindexer(Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, 0); //Change inits for comp

                robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);
        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        shooterManualControl = new FSMShooter(gamepadCo1, gamepadCo2, robot);
        shooterManualControl.Init();

        intake = new Intake(gamepadCo1, gamepadCo2, robot, spindexer, gamepadManager);

    }

    @Override
    public void loop() {
        //robotDrive.DriveLoop();
        gamepadManager.loop();

        shooterManualControl.ShooterLoop();
        intake.loop();
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
