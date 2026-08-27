package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Sample Command Based OpMode")
@Disabled
public class SampleScheduledOpmode extends OpMode {
    Action rootAction; //Set this to desired action

    @Override
    public void init(){
        //Setup command tree here
    }

    @Override
    public void start() {
        rootAction.init();
    }

    @Override
    public void loop(){
        rootAction.loop();
    }

    @Override
    public void stop(){
        rootAction.shutdown();
    }
}
