package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Optional;

@Config
@TeleOp (name = "Limelight 🐧")
public class LimeLightTeleOp extends OpMode {

    private FtcDashboard ftcDashboard;
    private Limelight limelight;
    private RobotHardware robot;
    private Turret turret;
    private VisionProcessing visionProcessing;

    private Pose2D filteredPose;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        turret = new Turret(robot);

        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        visionProcessing = new VisionProcessing();

        limelight = new Limelight(robot, turret);
        limelight.initLimelight(24);
        limelight.start();
    }

    @Override
    public void loop() {
        robot.pinpoint.update();

        Limelight.Output output = limelight.normalizedPose2D(DistanceUnit.MM);

        if (output.visionPose != null) {
            VisionProcessing.Stats stats = visionProcessing.updateAndGet(robot.pinpoint.getPosition(), output.visionPose);

            if (stats.filtered) {
                filteredPose = new Pose2D(DistanceUnit.MM, stats.meanRxMm, stats.meanRyMm, AngleUnit.DEGREES, output.visionPose.getHeading(AngleUnit.DEGREES));
            }
            else {
                filteredPose = output.visionPose;
            }

            telemetry.addData("limelight pose", "%.2f, %.2f, %.2f", output.visionPose.getX(DistanceUnit.MM), output.visionPose.getY(DistanceUnit.MM), output.visionPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("pinpoint pose", output.pinpointPose);
            telemetry.addData("fusedPose", "%.2f, %.2f, %.2f", output.fusedPose.getX(DistanceUnit.MM), output.fusedPose.getY(DistanceUnit.MM), output.fusedPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("tx", output.tx);
            telemetry.addData("ty", output.ty);
            telemetry.addData("quality", output.quality);
            telemetry.addData("alpha", output.alpha);
            telemetry.addData("error", output.error);
            telemetry.addData("staleness", output.staleness);
            telemetry.addData("tagCount", output.tagCount);
            telemetry.addData("didReset", output.didReset);
            telemetry.addLine("--------------------------------");
            telemetry.addData("n", stats.n);
            telemetry.addData("mean x", stats.meanRxMm);
            telemetry.addData("mean y", stats.meanRyMm);
            telemetry.addData("standard deviation x", stats.stdRxMm);
            telemetry.addData("standard deviation y", stats.stdRyMm);
            telemetry.addData("standard deviation distance", stats.stdRmm);
            telemetry.addData("valid", stats.valid);
            telemetry.addData("filtered", stats.filtered);

            telemetry.addData("filtered pose", "%.2f, %.2f, %.2f", filteredPose.getX(DistanceUnit.MM), filteredPose.getY(DistanceUnit.MM), filteredPose.getHeading(AngleUnit.DEGREES));

        }

        telemetry.update();

    }
}
