package org.firstinspires.ftc.teamcode.GamepadDriver;

import java.util.function.Supplier;

public interface GamepadInput<T> extends Supplier<T> {
    default void update(){};
}
