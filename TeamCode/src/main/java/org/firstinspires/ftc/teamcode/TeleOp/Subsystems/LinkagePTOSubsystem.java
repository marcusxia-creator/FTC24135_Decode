package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotActionConfig.*;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

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
