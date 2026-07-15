package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.SHOOTER_RPM_CONVERSION;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.PoseStorage;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;

import java.util.List;

public class TeleOpTelemetryManager {
    private final Telemetry telemetry;
    private int counter = 0;
    private int updateEveryLoops = 5;

    public TeleOpTelemetryManager(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setUpdateEveryLoops(int loops) {
        this.updateEveryLoops = Math.max(1, loops);
    }

    public void update(
            BasicTeleOp_RED_ALLIANCE.Alliance alliance,
            BasicTeleOp_RED_ALLIANCE.RobotActionState requestedActionState,
            BasicTeleOp_RED_ALLIANCE.RobotActionState activeActionState,
            List<String> switchTickLog,
            ColorDetection colorDetection,
            FSMIntake fsmIntake,
            FSMShooter fsmShooter,
            SpindexerUpd spindexer,
            RobotHardware robot,
            ShooterPowerCalculator shooterPowerAngleCalculator,
            Turret turret,
            Limelight limelight,
            double loopHz
    ) {
        counter++;

        if (counter % updateEveryLoops != 0) {
            return;
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Loop Hz", "%.1f", loopHz);
        telemetry.addData("voltage from robot", robot.getBatteryVoltageRobust());

        telemetry.addLine("-----ROBOT STATE-----");
        telemetry.addData("Requested", requestedActionState);
        telemetry.addData("Active", activeActionState);
        telemetry.addData("IntakeState", fsmIntake.intakeStates);
        telemetry.addData("ShooterState", fsmShooter.shooterState);
        telemetry.addData("IntakeSafe", fsmIntake.canExit());
        telemetry.addData("ShooterSafe", fsmShooter.canExit());

        telemetry.addLine("-----SPINDEXER-----");
        telemetry.addData("Distance Sensor", robot.distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Sensor Color", colorDetection.getStableColor());
        telemetry.addData("Sensor values", spindexer.colorValue);
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addData("Current Pos", spindexer.currentPos);
        telemetry.addData("Current Index", spindexer.index);

        telemetry.addLine("-----SHOOTER-----");
        telemetry.addData("distance to goal", "%,.0f",shooterPowerAngleCalculator.getDistance());
        telemetry.addData("Shooter Zone", shooterPowerAngleCalculator.getCurrentZone());
        telemetry.addData("shooter power calculator", shooterPowerAngleCalculator.getPower());
        telemetry.addData("Shooter actual Power", robot.topShooterMotor.getPower());
        telemetry.addData("voltage from Shooter", fsmShooter.getVoltage());
        telemetry.addData("Shooter Target RPM",shooterPowerAngleCalculator.getRPM());
        telemetry.addData("Shooter acutal RPM",shooterPowerAngleCalculator.getMeasureRPM());
        telemetry.addData("Shooter RPM","%,.0f",robot.topShooterMotor.getVelocity()*SHOOTER_RPM_CONVERSION);

        Pose2D pose = robot.pinpoint.getPosition();

        telemetry.addLine("-----POSE-----");
        telemetry.addData(
                "Pose",
                "X: %.2f  Y: %.2f  H: %.1f",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                Math.toDegrees(pose.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS))
        );

        telemetry.addLine("-----TURRET-----");
        telemetry.addData("Limit Switch State", robot.limitSwitch.getState());
        telemetry.addData("Limit Switch Log", switchTickLog.toString());
        telemetry.addData("goal pose", turret.getGoalPose());
        telemetry.addData("turret target angle", turret.getTargetAngle());
        telemetry.addData("turret drive angle", turret.getTurretDriveAngle());
        telemetry.addData("turret motor angle", turret.getTurretMotorAngle());
        telemetry.addData("target motor tick", turret.getTargetTick());
        telemetry.addData("current motor tick", turret.getCurrentTick());
        telemetry.addData("turret auto end tick", PoseStorage.turretEndTick);
        telemetry.addData("turret offset tick", turret.getTurretOffsetTick());
        telemetry.addData("turret shooting mode", fsmShooter.turretState);
        telemetry.addData("turret power", robot.turretMotor.getPower());

        telemetry.addLine("-----LIMELIGHT-----");
        telemetry.addData("Tx", limelight.getTargetXForTag(24));
        telemetry.addData("Green Slot", limelight.getGreenSlot());

        telemetry.update();
    }

    public void updateSimplified(
            BasicTeleOp_RED_ALLIANCE.Alliance alliance,
            BasicTeleOp_RED_ALLIANCE.RobotActionState requestedActionState,
            BasicTeleOp_RED_ALLIANCE.RobotActionState activeActionState,
            FSMIntake fsmIntake,
            FSMShooter fsmShooter,
            SpindexerUpd spindexer,
            RobotHardware robot,
            ShooterPowerCalculator shooterPowerAngleCalculator,
            Turret turret,
            Limelight limelight,
            double loopHz
    ) {
        telemetry.addLine("-----SPINDEXER-----");
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addLine("-----ROBOT STATE-----");
        telemetry.addData("Requested", requestedActionState);
        telemetry.addData("Active", activeActionState);
        telemetry.addData("IntakeState", fsmIntake.intakeStates);
        telemetry.addData("ShooterState", fsmShooter.shooterState);
        telemetry.addData("IntakeSafe", fsmIntake.canExit());
        telemetry.addData("ShooterSafe", fsmShooter.canExit());
        telemetry.addData("distance to goal", shooterPowerAngleCalculator.getDistance());
        String MotifEnabled;
        String MotifAvailable;
        telemetry.addLine("-----SHOOTER STATE-----");
        telemetry.addData("Shooter Target Colour", fsmShooter.targetColour.name());
        telemetry.addData("Shooter Power-LUT OUT", robot.topShooterMotor.getPower());
        telemetry.update();
    }
}
