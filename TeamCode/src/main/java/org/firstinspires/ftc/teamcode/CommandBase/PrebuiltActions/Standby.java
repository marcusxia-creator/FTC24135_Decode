package org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions;

import org.firstinspires.ftc.teamcode.CommandBase.Action;

public class Standby implements Action {
    boolean finishedReturn;

    public Standby(boolean finished){
        finishedReturn=finished;
    }
    public Standby(){
        finishedReturn=false;
    }

    @Override
    public void init() {}

    @Override
    public void loop() {}

    @Override
    public void shutdown() {}

    @Override
    public boolean finished() {
        return finishedReturn;
    }
}
