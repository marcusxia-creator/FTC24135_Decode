package CommandBase;

public interface Action {
    void init();///Runs once to start the action
    void loop();///Runs continuously once action is started
    boolean finished();///Returns whether the action is completed or not
    void shutdown();///Runs when action should end, either by force or when `finished()` returns true. Distinguish forced shutdowns and smooth shutdowns by running 'finished()' locally.
}
