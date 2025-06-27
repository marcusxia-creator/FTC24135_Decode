package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Point;

public class advancedIntake {
    public static void runSlides(RobotHardware robot, double slideExtension, DistanceUnit unit){
        if(unit==DistanceUnit.CM){slideExtension=slideExtension/2.54;unit=DistanceUnit.INCH;}

        double a=RobotActionConfig.slide_Arm_A_Length;
        double b=RobotActionConfig.slide_Arm_B_Length;

        double minBase=8.3;

        double base=slideExtension+minBase;
        double theta=Math.acos(
                (Math.pow(base,2)+Math.pow(a,2)-Math.pow(b,2))/
                        (2*a*base)
        );
        double servoWrite=(Math.acos(
                (Math.pow(minBase,2)+Math.pow(a,2)-Math.pow(b,2))/
                        (2*a*minBase)
        )-theta)/Math.toRadians(300);

        robot.intakeLeftSlideServo.setPosition(servoWrite);
        robot.intakeRightSlideServo.setPosition(servoWrite);
    }

    public static void runToPoint(RobotHardware robot, Point point, DistanceUnit unit){
        if(unit==DistanceUnit.CM){point.x=point.x/2.54;point.y=point.y/2.54;}

        double arm=RobotActionConfig.Turret_Arm_Length;

        double turret=Math.asin(point.x/arm);
        double slideExtension=(point.y-arm*Math.cos(Math.asin(point.x/arm)));

        robot.intakeTurretServo.setPosition(0.31*(turret/Math.PI)+0.31);
        runSlides(robot,slideExtension,DistanceUnit.INCH);
    }
}