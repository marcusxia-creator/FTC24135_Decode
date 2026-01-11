package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.LUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class LUTPowerCalculator {

    private RobotHardware robot;
    private PIDController pidController;

    private double distance;
    private int zone = 0;

    private double tickToRPM = (double) 60 / 28;

    private int maxVelocity = 5500;

    //Stores the position of both alliance's goal
    private final Pose2D redGoalPose = new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, -45);
    private final Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, -70, -70, AngleUnit.DEGREES, 45);

    //The actual goal pose (depends on which alliance)
    private Pose2D actualGoalPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    //Sets the pid value
    private final double p = 5, i = 0, d = 0;

    //Target RPM base on the zone
    LUT<Integer, Integer> targetRPM = new LUT<Integer, Integer>()
    {{
        add(4, (int) (5000*FZPower));//far zone   0.93*4800
        add(3, (int) (5000*farPower));//far range   0.78*4800
        add(2, (int) (5000*midPower)); //mid range  0.73*4800
        add(1, (int) (5000*closePower)); //near range 0.75*4800
        add(0, 3000); //Not in shooting zone

    }};

    //Target shooting angle base on the zone
    LUT<Integer, Double> targetShootingAngle = new LUT<Integer, Double>()
    {{
        add(3, 0.4);//far zone
        add(2, 0.5); //mid zone
        add(1, 0.6); //near zone
        add(0, 0.3); //Not in shooting zone

    }};

    //Constructor for parameter setup
    public LUTPowerCalculator(RobotHardware robot) {
        this.robot = robot;
        pidController = new PIDController(p, i, d);
    }

    //Set the alliance of the match which determine the actual goal pose
    public void setAlliance (boolean isRedAlliance) {
        if (isRedAlliance) {
            actualGoalPose = redGoalPose;
        }

        if (!isRedAlliance) {
            actualGoalPose = blueGoalPose;
        }
    }

    //Returns the distance of the robot to the goal
    public double getDistance() {
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH), 2));
        return distance;
    }

    //Returns the power for the shooter motors
    public double getPower() {
        //Calculate the distance of the robot from goal base on pythagoras theorem
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH), 2));

        //Determine which zone the robot is currently in
        if (distance > CLOSE && distance <= MID) {
            zone = 1;
        }
        else if (distance > MID && distance <= FAR) {
            zone = 2;
        }
        else if (distance > FAR && distance <= FAR_EDGE) {
            zone = 3;
        }
        else if (distance > FAR_ZONE_LOW && distance <= FAR_ZONE_HIGH) {
            zone = 4;
        }
        else {
            zone = 0;
        }

        //Normalize target and current values
        double current = (robot.topShooterMotor.getVelocity() * tickToRPM) / maxVelocity;
        double target = targetRPM.get(zone) / maxVelocity; //Uses zone to get target rpm

        //Returns the power for the motor base on PID controller
        return pidController.calculate(current, target);
    }

    public double getAngle() {
        double tangent = (actualGoalPose.getY(DistanceUnit.INCH) - robot.pinpoint.getPosY(DistanceUnit.INCH)) / distance;
        return Math.toDegrees(Math.atan(tangent));
    }

    //Returns the desired angle for shooter
    public double getShooterAngle() {
        //Calculate distance of robot to goal
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH), 2));

        //Determine the zone the robot is in
        if (distance > CLOSE && distance <= MID) {
            zone = 1;
        }
        else if (distance > MID && distance <= FAR) {
            zone = 2;
        }
        else if (distance > FAR && distance <= FAR_EDGE) {
            zone = 3;
        }
        else {
            zone = 0;
        }

        //Returns the target angle
        return targetShootingAngle.get(zone);
    }
}
