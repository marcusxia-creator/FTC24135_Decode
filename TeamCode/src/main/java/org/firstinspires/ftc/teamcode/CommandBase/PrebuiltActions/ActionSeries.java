package org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions;

import org.firstinspires.ftc.teamcode.CommandBase.Action;

public class ActionSeries implements Action {
    Action[] actionList;
    int index;

    public ActionSeries(Action... actions){
        actionList=actions;
        index=0;
    }

    public Action[] getActionList() {
        return actionList;
    }

    public void setActionList(Action... actions){
        actionList=actions;
    }

    @Override
    public void init() {
        index=0;
        actionList[0].init();
    }

    @Override
    public void loop() {
        if(index<actionList.length) {
            actionList[index].loop();
            if(actionList[index].finished()){//Only move on to next action if there is one left in the queue
                actionList[index].shutdown();
                index+=1;
                if(index<actionList.length){
                    actionList[index].init();
                }
            }
        }
    }


    @Override
    public void shutdown(){
        actionList[index].shutdown();
    }

    @Override
    public boolean finished() {
        return index>actionList.length;
    }
}
