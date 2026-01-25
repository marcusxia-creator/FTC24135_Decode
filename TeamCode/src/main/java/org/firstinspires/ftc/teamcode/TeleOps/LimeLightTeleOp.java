package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class LimeLightTeleOp extends OpMode {

    private Limelight limelight;
    private RobotHardware robot;
    private Turret turret;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        turret = new Turret(robot);

        limelight = new Limelight(robot, turret);
        limelight.initLimelight(24);
        limelight.start();
    }

    @Override
    public void loop() {
        Limelight.Output output = limelight.normalizedPose2D(DistanceUnit.MM);

        if (output != null) {
            telemetry.addData("limelight output", output);
            telemetry.update();
        }
    }
}
