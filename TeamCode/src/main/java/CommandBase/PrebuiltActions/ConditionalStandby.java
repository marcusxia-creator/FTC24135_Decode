package CommandBase.PrebuiltActions;

import java.util.function.Supplier;

import CommandBase.Action;

public class ConditionalStandby implements Action {
    Supplier<Boolean> condition;

    public ConditionalStandby(Supplier<Boolean> condition){
        this.condition=condition;
    }

    @Override
    public void init() {}

    @Override
    public void loop() {}

    @Override
    public void shutdown() {}

    @Override
    public boolean finished() {
        return condition.get();
    }
}
