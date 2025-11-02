package org.firstinspires.ftc.teamcode.TeleOps;

public class SpindexerManualControl {
    RobotHardware robot;
    Spindexer spindexer;
    GamepadManager gamepadManager;
    public SpindexerManualControl(RobotHardware robot, Spindexer spindexer, GamepadManager gamepadManager){
        this.robot=robot;
        this.spindexer=spindexer;
        this.gamepadManager=gamepadManager;
    }
    public void loop() {
        if(gamepadManager.Unjam.PressState){
            spindexer.unJam();
        }
        if(gamepadManager.spinPrev.PressState){
            spindexer.runToSlot(spindexer.currentSlot-1);
        }
        if(gamepadManager.spinNext.PressState){
            spindexer.runToSlot(spindexer.currentSlot+1);
        }

        if(gamepadManager.reDetc.PressState){
            spindexer.writeToCurrent(robot.colorSensor, robot.distanceSensor);
        }
    }
}
