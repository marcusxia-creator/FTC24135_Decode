package CommandBase.PrebuiltActions;

import java.util.ArrayList;
import java.util.Arrays;

import CommandBase.Action;

public class ActionSeries implements Action {
    ArrayList<Action> actionList;

    public ActionSeries(Action... actions){
        actionList=new ArrayList<>(Arrays.asList(actions));
    }

    public ArrayList<Action> getActionList() {
        return actionList;
    }

    public void setActionList(Action... actions){
        actionList=new ArrayList<>(Arrays.asList(actions));
    }

    public void setActionList(ArrayList<Action> actionList){
        this.actionList=actionList;
    }

    @Override
    public void init() {
        actionList.get(0).init();
    }

    @Override
    public void loop() {
        actionList.get(0).loop();
        if(actionList.size()>1&&actionList.get(0).finished()){//Only move on to next action if there is one left in the queue
            actionList.get(0).shutdown();
            actionList.remove(0);
            actionList.get(0).init();
        }}


    @Override
    public void shutdown(){
        actionList.get(0).shutdown();
    }

    @Override
    public boolean finished() {
        return actionList.size()==1 && actionList.get(0).finished();
    }
}
