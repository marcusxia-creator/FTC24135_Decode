package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing;

public enum ArcDirection {
    CLOCKWISE (1),
    COUNTERCLOCKWISE (-1);

    private final int factor;

    private ArcDirection(int factor){
        this.factor=factor;
    }

    public int getFactor() {
        return factor;
    }
}
