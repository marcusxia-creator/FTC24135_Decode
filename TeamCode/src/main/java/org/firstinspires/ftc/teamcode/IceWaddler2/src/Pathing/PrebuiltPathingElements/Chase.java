package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.Movement;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Supplier;

public class Chase implements Movement {
    Position targetPosition;
    Queue<Position> lastTargetPositions=new LinkedList<>();
    Supplier<Position> positionSupplier;
    boolean dynamicStartpoint;
    String[] tags;
    Scalar dt;

    Position relPos;

    public Chase(Supplier<Position> positionSupplier, String[] tags){
        this.positionSupplier=positionSupplier;
        dynamicStartpoint=false;
        this.tags=tags;
    }

    public Chase(String[] tags){
        this(null,tags);
    }

    @Override
    public void init(PathingPoint lastTargetPoint) {
        targetPosition=positionSupplier.get();
    }

    @Override
    public void loop(Situation currentSituation, Scalar dt) {
        targetPosition=positionSupplier.get();
        lastTargetPositions.offer(targetPosition);
        if(lastTargetPositions.size()>derivativeTicks){
            lastTargetPositions.poll();
        }
        targetPosition=positionSupplier.get();
        relPos=currentSituation.getPosition().sub(targetPosition);
        this.dt=dt;
    }

    @Override
    public Velocity getTargetVel() {
        return targetPosition.sub(lastTargetPositions.peek()).differentiate(dt)
                .add(new Velocity(
                relPos.getLinPos().unitVector().multi(latPosController.getCorrection(relPos.getLinPos().mag())),
                headingController.getCorrection(relPos.getHeading()))
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
        return new PathingPoint(targetPosition,new Scalar(0,metersPerSecond));
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
