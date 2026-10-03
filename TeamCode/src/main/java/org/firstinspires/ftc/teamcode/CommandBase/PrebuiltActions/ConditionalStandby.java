package org.firstinspires.ftc.teamcode.CommandBase.PrebuiltActions;

import java.util.function.Supplier;

import org.firstinspires.ftc.teamcode.CommandBase.Action;

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
