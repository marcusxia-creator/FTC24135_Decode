package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltPathingElements;

import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Dimensions;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.*;

public class FixedArc implements Movement {

    PathingPoint startPoint;
    Vector center;
    ArcDirection direction;
    Scalar endVelocity;
    MotionProfile motionProfile;
    HeadingProfile headingProfile;
    String[] tags;
    boolean dynamicStartpoint;

    //Pathing helpers
    NormalizedAngle currentglobalAngle;
    NormalizedAngle initialGlobalAngle;
    Scalar currentRadius;
    Scalar targetRadius;
    PositiveAngle currentTheta;
    PositiveAngle targetTheta;
    Scalar distanceTraveled;
    Scalar totalDistance;
    PathingPoint endPoint;
    NormalizedAngle endHeading;

    NormalizedAngle currentHeading;
    NormalizedAngle lastTargetHeading;
    Scalar tickTime;

    public FixedArc(PathingPoint startPoint, Vector center, PositiveAngle angle, ArcDirection direction, Scalar endVelocity, NormalizedAngle endHeading, MotionProfile motionProfile, HeadingProfile headingProfile, String[] tags){
        this.startPoint = startPoint;
        if(!center.getDimensions().equals(Dimensions.length)){throw new DimMismatch(center.getDimensions(),"center point");}
        this.targetTheta=angle;
        this.direction=direction;
        if(!endVelocity.getDimensions().equals(Dimensions.velocity)){throw new DimMismatch(endVelocity.getDimensions(),"end velocity");}
        this.endVelocity=endVelocity;
        this.endHeading=endHeading;
        this.motionProfile = motionProfile;
        this.headingProfile = headingProfile;
        this.tags = tags;
        dynamicStartpoint=false;
    }

    public FixedArc(Vector center, PositiveAngle angle, ArcDirection direction, Scalar endVelocity, NormalizedAngle endHeading, MotionProfile motionProfile, HeadingProfile headingProfile, String[] tags){
        this(null, center, angle, direction, endVelocity, endHeading, motionProfile, headingProfile, tags);
        dynamicStartpoint=true;
    }

    @Override
    public void init(PathingPoint lastPathingPoint){
        if(dynamicStartpoint){
            startPoint=lastPathingPoint;
        }
        //Init Line Params
        targetRadius=startPoint.getPosition().getLinPos().sub(center).mag();
        totalDistance=targetRadius.multiply(targetTheta.div(new Scalar(1,rad)));
        initialGlobalAngle=center.angleTo(startPoint.getPosition().getLinPos());
        endPoint=new PathingPoint(new Position(startPoint.getPosition().getLinPos().sub(center).rotateBy(targetTheta.toNormalizedAngle().multiply(direction.getFactor())).add(center),endHeading),
                endVelocity);

        //Init profiles
        motionProfile.init(startPoint.getVelocity(),endVelocity,totalDistance);
        headingProfile.init(startPoint.getPosition().getAngPos(),endHeading,totalDistance);

        lastTargetHeading=startPoint.getPosition().getHeading();
    }

    @Override
    public PathingPoint getTargetPoint(){
        return endPoint;
    }

    @Override
    public void loop(Situation currentSituation, Scalar tickTime) {
        currentglobalAngle=center.angleTo(currentSituation.getPosition().getLinPos());
        currentTheta=currentglobalAngle.sub(initialGlobalAngle).multiply(direction.getFactor()).toPositiveAngle();
        currentRadius=currentSituation.getPosition().getLinPos().sub(center).mag();
        currentHeading=currentSituation.getPosition().getHeading();
        this.tickTime=tickTime;
    }

    @Override
    public Velocity getTargetVel() {
        NormalizedAngle targetHeading=headingProfile.getAng(getCompletion());
        Velocity targetVel=new Velocity(new Vector(latPosController.getCorrection(targetRadius.sub(currentRadius)),
                motionProfile.getVel(getCompletion()).multiply(direction.getFactor()))//x is radial correction (+ is inward), y is clockwise velocity
                .rotateBy(currentglobalAngle.add(new NormalizedAngle(90,deg))),
                targetHeading.sub(lastTargetHeading).div(tickTime).add(headingController.getCorrection(currentHeading.sub(targetHeading))));
        lastTargetHeading=targetHeading;
        return targetVel;
    }

    @Override
    public Scalar getDistanceTravelled() {
        return targetRadius.multiply(currentTheta.div(new Scalar(1,rad)));
    }

    @Override
    public double getCompletion() {
        return getDistanceTravelled().div(totalDistance).getValueSI();
    }

    @Override
    public boolean finished() {
        return totalDistance.sub(getDistanceTravelled()).lessThanOrEqual(distThreshold);
    }

    @Override
    public String[] getTags() {
        return tags;
    }
}
