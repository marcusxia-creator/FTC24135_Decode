package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Subsystems.UniversalTools.RobotHardware;

public class LinkagePTOSubsystem extends SubsystemBase {
    private RobotHardware robot;
    public LinkagePTOSubsystem (RobotHardware robot) {
        this.robot = robot;
    }
    public void engagePTO (){
        robot.linkagePTOServo.setPosition(engagePTO);
    }
    public void disengagePTO (){
        robot.linkagePTOServo.setPosition(disengagePTO);
    }
}
