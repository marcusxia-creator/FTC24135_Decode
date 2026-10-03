package org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions;

import org.firstinspires.ftc.teamcode.CommandBase.Action;

public class ActionParallel implements Action {
    public enum TERMINATIONTYPE{
        FIRST,
        ALL,
        NONE
    }

    TERMINATIONTYPE terminationType;
    Action[] actionList;
    public ActionParallel(TERMINATIONTYPE terminationType, Action... actions) {
        this.terminationType=terminationType;
        actionList=actions;
    }

    public Action[] getActionList() {
        return actionList;
    }

    public void setActionList(Action... actions){
        actionList=actions;
    }

    //Implementations
    @Override
    public void init(){
        for(Action action:actionList){
            action.init();
        }
    }

    @Override
    public void loop(){
        for(Action action:actionList){
            action.loop();
        }
    }

    @Override
    public void shutdown(){
        for(Action action:actionList){
            action.shutdown();
        }
    }

    @Override
    public boolean finished() {
        if (terminationType==TERMINATIONTYPE.FIRST){
            for (Action action : actionList) {
                if (action.finished()){
                    return true; //If any single action is finished, whole action is finished
                }
            }
            return false;
        }
        else if(terminationType==TERMINATIONTYPE.ALL){
            for(Action action:actionList){
                if(!action.finished()){
                    return false; //If any single action is not finished, whole action must not be finished
                }
            }
            return true;
        }
        else{
            return false; //Action is never finished if `terminationType==NONE`
        }
    }
}
