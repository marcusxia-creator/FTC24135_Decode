package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class FSMIntakeAuto {
    ///FSM States
    public enum INTAKESTATE {
        INTAKE_READY,
        INTAKE_RUN,
        INTAKE_DETECT,
        INTAKE_INDEX,
        INTAKE_SPIN,
        INTAKE_END,
        INTAKE_UNJAM
    }

    ///Variables
    private INTAKESTATE currentState;
    private final RobotHardware robot;
    private final RobotActionConfig robotActionConfig;

    ///Constructor
    public FSMIntakeAuto (RobotHardware robot, RobotActionConfig robotActionConfig){
        this.robot = robot;
        this.robotActionConfig = robotActionConfig;
    }

    public void FSMRun(){
        switch (currentState){
            case INTAKE_READY:
                robot.init();
                currentState = INTAKESTATE.INTAKE_RUN;
                break;

            case INTAKE_RUN:
                robot.intakeMotor.setPower(RobotActionConfig.intakeSpeed);
                break;
            case INTAKE_DETECT:
                break;
            case INTAKE_INDEX:
                break;
            case INTAKE_SPIN:
                break;

        }
    }
}
