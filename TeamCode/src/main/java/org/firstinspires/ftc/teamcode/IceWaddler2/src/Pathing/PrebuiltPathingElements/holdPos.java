package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.Movement;

public class holdPos implements Movement {
    Position position;
    boolean dynamicStartpoint;
    String[] tags;

    Position relPos;

    public holdPos(Position position, String[] tags){
        this.position=position;
        dynamicStartpoint=false;
        this.tags=tags;
    }

    public holdPos(String[] tags){
        this(null,tags);
    }

    @Override
    public void init(PathingPoint lastTargetPoint) {
        if(dynamicStartpoint){
            position=lastTargetPoint.getPosition();
        }
    }

    @Override
    public void loop(Situation currentSituation, Scalar tickTime) {
        relPos=currentSituation.getPosition().sub(position);
    }

    @Override
    public Velocity getTargetVel() {
        return new Velocity(
                relPos.getLinPos().unitVector().multi(latPosController.getCorrection(relPos.getLinPos().mag())),
                headingController.getCorrection(relPos.getHeading())
        );
    }

    @Override
    public Scalar getDistanceTravelled() {
        return new Scalar(0,m);
    }

    @Override
    public boolean finished() {
        return relPos.getLinPos().mag().lessThanOrEqual(distThreshold)&&abs(relPos.getHeading().getValueSI())<=angThreshold.getValueSI();
    }

    @Override
    public PathingPoint getTargetPoint() {
        return new PathingPoint(position,new Scalar(0,metersPerSecond));
    }

    @Override
    public double getCompletion() {
        return finished()?1:0;
    }

    @Override
    public String[] getTags() {
        return tags;
    }
}
