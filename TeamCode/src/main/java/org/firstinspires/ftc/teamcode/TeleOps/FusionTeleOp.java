package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp (name = "Limelight Fusion")
public class FusionTeleOp extends OpMode {

    private FtcDashboard ftcDashboard;
    private RobotHardware robot;
    private Turret turret;
    private LimelightFusion limelightFusion;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        turret = new Turret(robot);

        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        limelightFusion = new LimelightFusion(robot, turret);
        limelightFusion.initLimelight(24);
        limelightFusion.start();
    }

    @Override
    public void loop() {
        robot.pinpoint.update();

        LimelightFusion.Output output = limelightFusion.normalizedPose2D(DistanceUnit.MM);

        if (output.visionPose != null) {

            telemetry.addData("limelight pose", "%.2f, %.2f, %.2f", output.visionPose.getX(DistanceUnit.MM), output.visionPose.getY(DistanceUnit.MM), output.visionPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pinpoint pose", "%.2f, %.2f, %.2f", output.pinpointPose.getX(DistanceUnit.MM), output.pinpointPose.getY(DistanceUnit.MM), output.pinpointPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("filteredPose", "%.2f, %.2f, %.2f", output.filteredPose.getX(DistanceUnit.MM), output.filteredPose.getY(DistanceUnit.MM), output.filteredPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("fusedPose", "%.2f, %.2f, %.2f", output.fusedPose.getX(DistanceUnit.MM), output.fusedPose.getY(DistanceUnit.MM), output.fusedPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("tx", output.tx);
            telemetry.addData("ty", output.ty);
            telemetry.addData("quality", output.quality);
            telemetry.addData("alpha", output.alpha);
            telemetry.addData("error", output.error);
            telemetry.addData("staleness", output.staleness);
            telemetry.addData("tagCount", output.tagCount);
            telemetry.addData("didReset", output.didReset);
        }

        telemetry.update();

    }
}