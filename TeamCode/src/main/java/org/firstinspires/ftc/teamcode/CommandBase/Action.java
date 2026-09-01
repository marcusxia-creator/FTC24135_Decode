package org.firstinspires.ftc.teamcode.CommandBase;

public interface Action {
    default void init(){};///Runs once to start the action
    void loop();///Runs continuously once action is started
    default boolean finished(){return false;};///Returns whether the action is completed or not
    default void shutdown(){};///Runs when action should end, either by force or when `finished()` returns true. Distinguish forced shutdowns and smooth shutdowns by running 'finished()' locally.
}
