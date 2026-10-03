package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class ScheduledOpMode extends OpMode{
    public Action rootAction;

    @Override
    abstract public void init();

    @Override
    public void start() {
        rootAction.init();
    }

    @Override
    public void loop() {
        rootAction.loop();
    }

    @Override
    public void stop() {
        rootAction.shutdown();
    }
}
