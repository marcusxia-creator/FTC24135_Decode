package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements;

import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.*;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.*;

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
    Scalar latError;
    NormalizedAngle lineAngle;

    //Pathing helpers
    Position relativePos;
    NormalizedAngle lastTargetHeading;

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

        lastTargetHeading=startPoint.getPosition().getHeading();
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
        NormalizedAngle targetHeading=headingProfile.getAng(getCompletion());
        Velocity targetVel=new Velocity(
                new Vector(latPosController.getCorrection(relativePos.getX()),
                        motionProfile.getVel(getCompletion())).rotateBy(lineAngle),
                targetHeading.sub(lastTargetHeading).div(tickTime).add(headingController.getCorrection(relativePos.getHeading().sub(targetHeading))));
        lastTargetHeading=targetHeading;
        return targetVel;
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
        return totalDistance.sub(getDistanceTravelled()).lessThanOrEqual(distThreshold);
    }

    @Override
    public String[] getTags(){
        return tags;
    }
}
