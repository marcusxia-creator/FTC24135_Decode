package org.firstinspires.ftc.teamcode.GamepadDriver;

import org.firstinspires.ftc.teamcode.CommandBase.Action;

import java.util.Arrays;

public abstract class GamepadDriver {
    public class Update implements Action{
         GamepadInput<?>[] updatableInputs;

         @Override
         public void init() {
             updatableInputs =Arrays.stream(this.getClass().getDeclaredFields())
                     .filter(field -> GamepadInput.class.isAssignableFrom(field.getType()))
                     .map(field -> {
                         try {
                             return field.get(this);
                         } catch (IllegalAccessException e) {
                             throw new RuntimeException(e);
                         }
                     })
                     .toArray(GamepadInput<?>[]::new);
         }

         @Override
         public void loop() {
             Arrays.stream(updatableInputs).forEach(GamepadInput::update);
         }
     }
}
