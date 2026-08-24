package CommandBase.PrebuiltActions;

import java.util.function.Supplier;

import CommandBase.Action;

public class ActivatableAction implements Action {
    Supplier<Boolean> activationMethod;
    Supplier<Boolean> deactivationMethod;
    Action action;
    boolean activated;
    boolean initialState;

    public ActivatableAction(Supplier<Boolean> activationMethod, Supplier<Boolean> deactivationMethod, Action action, boolean initialState){
        this.activationMethod=activationMethod;
        this.deactivationMethod=deactivationMethod;
        this.action=action;
        this.initialState=initialState;
    }

    public ActivatableAction(Supplier<Boolean> activationMethod, Supplier<Boolean> deactivationMethod, Action action){
        this(activationMethod,deactivationMethod,action,false);
    }

    public void activate(){
        action.init();
        activated=true;
    }

    public void deactivate(){
        action.shutdown();
        activated=false;
    }

    @Override
    public void init(){
        activated=initialState;
        if(activated){
            activate();
        }
    }

    @Override
    public void loop(){
        if(activationMethod.get()&&!activated){
            activate();
        }
        else if((deactivationMethod.get())&&activated){
            deactivate();
        }

        if(activated){
            action.loop();
        }
    }

    @Override
    public void shutdown() {
        deactivate();
    }

    @Override
    public boolean finished() {
        return !activated||action.finished();
    }
}
