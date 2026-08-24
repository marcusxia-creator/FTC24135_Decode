package CommandBase.PrebuiltActions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import CommandBase.Action;

public class ActionSwitch implements Action{
    Supplier<Integer> switchPos;
    Action[] actionList;
    int lastPos;

    public ActionSwitch(Supplier<Integer> switchPosMethod, Action... actions){
        this.switchPos=switchPosMethod;
        this.actionList=actions;
    }

    public Action[] getActionList() {
        return actionList;
    }

    public void setActionList(Action... actions){
        actionList=actions;
    }

    @Override
    public void init(){
        actionList[switchPos.get()].init();
        lastPos=this.switchPos.get();
    }

    @Override
    public void loop(){
        int currentPos=switchPos.get();
        //Switch if needed
        if(currentPos!=lastPos){
            if(0<=lastPos&&lastPos<actionList.length) {
                actionList[lastPos].shutdown();
            }
            if(0<=currentPos&&currentPos<actionList.length) {
                actionList[currentPos].init();
            }
            lastPos=currentPos;
        }
        if(0<=currentPos&&currentPos<actionList.length) {
            actionList[currentPos].loop();
        }
    }

    @Override
    public void shutdown() {
        actionList[switchPos.get()].shutdown();
    }

    @Override
    public boolean finished() {
        return actionList[switchPos.get()].finished();
    }
}
