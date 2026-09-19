package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements;

import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.*;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.*;

import java.util.LinkedList;
import java.util.Queue;

public class Line implements Movement {
    PathingPoint startPoint;
    PathingPoint endPoint;
    MotionProfile motionProfile;
    HeadingProfile headingProfile;
    String[] tags;
    boolean dynamicStartpoint;

    Scalar tickTime;

    //LineParams
    Scalar totalDistance;
    NormalizedAngle lineAngle;

    //Pathing helpers
    Position relativePos;

    public Line(PathingPoint startPoint, PathingPoint endPoint, MotionProfile motionProfile, HeadingProfile headingProfile, String[] tags) {
        this.startPoint = startPoint;
        this.motionProfile = motionProfile;
        this.headingProfile = headingProfile;
        this.endPoint = endPoint;
        this.tags = tags;
        dynamicStartpoint=false;
    }

    public Line(PathingPoint endPoint, MotionProfile motionProfile, HeadingProfile headingProfile, String[] tags) {
        this(null,endPoint,motionProfile,headingProfile,tags);
        dynamicStartpoint=true;
    }

    @Override
    public void init(PathingPoint lastPathingPoint){
        if(dynamicStartpoint){
            startPoint=lastPathingPoint;
        }
        //Init Line Params
        totalDistance=endPoint.getPosition().sub(startPoint.getPosition()).mag();
        lineAngle=startPoint.getPosition().getLinPos().angleTo(endPoint.getPosition().getLinPos());

        //Init profiles
        motionProfile.init(startPoint.getVelocity(),endPoint.getVelocity(),totalDistance);
        headingProfile.init(startPoint.getPosition().getAngPos(),endPoint.getPosition().getAngPos(),totalDistance);
    }

    @Override
    public PathingPoint getTargetPoint(){
        return endPoint;
    }

    @Override
    public void loop(Situation currentSituation, Scalar tickTime) {
        relativePos=new Position(currentSituation.getPosition().sub(startPoint.getPosition()).getLinPos().rotateBy(lineAngle.multiply(-1)),
                currentSituation.getPosition().getAngPos());//x is error, y is distance along line
        this.tickTime=tickTime;
    }

    @Override
    public Velocity getTargetVel(){

        Scalar MPvel=motionProfile.getVel(getCompletion());

        return new Velocity(
                new Vector(latPosController.getCorrection(relativePos.getX()),
                        MPvel).rotateBy(lineAngle),
                headingProfile.getAngVel(getCompletion(),MPvel).add(headingController.getCorrection(relativePos.getHeading().sub(headingProfile.getHeading(getCompletion())))));
    }

    @Override
    public Scalar getDistanceTravelled(){
        return relativePos.getY();
    }

    @Override
    public double getCompletion(){
        return getDistanceTravelled().div(totalDistance).getValueSI();
    }

    @Override
    public boolean finished(){
        return getDistanceTravelled().greaterThanOrEqual(endPoint.getVelocity().lessThan(minSpeed)?totalDistance.sub(distThreshold):totalDistance);
    }

    @Override
    public String[] getTags(){
        return tags;
    }
}
