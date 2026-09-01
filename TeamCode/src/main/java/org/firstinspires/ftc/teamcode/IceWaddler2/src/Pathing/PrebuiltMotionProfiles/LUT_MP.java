package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.MotionProfile;

public class LUT_MP implements MotionProfile{
    double[] completionNodes;
    Scalar[] velNodes;
    Scalar startVel;
    Scalar endVel;
    Scalar totalDistance;
    MotionProfile[] interpolators; //The interpolation motion profile that is run between nodes. Entering an interpolators list of length 1 will reuse the same interpolator for all interpolations. Otherwise, the interpolators list should be 1 element longer than each of the nodes lists

    int totalBins;
    int currentBin;
    double currentBinSize;

    public LUT_MP(double[] completionNodes, Scalar[] velNodes, MotionProfile[] interpolators){
        this.completionNodes=completionNodes;
        this.velNodes=velNodes;
        this.interpolators=interpolators;
        this.totalBins =completionNodes.length+1;
    }

    @Override
    public void init(Scalar startVel, Scalar endVel, Scalar totalDistance) {
        this.startVel=startVel;
        this.endVel=endVel;
        currentBin=0;
    }

    double getLastCompletionNode(){
        return currentBin==0?0:completionNodes[currentBin-1];
    }

    double getNextCompletionNode(){
        return currentBin>=totalBins-1?1:completionNodes[currentBin];
    }

    Scalar getStartVel(){
        return currentBin==0?startVel:velNodes[currentBin-1];
    }

    Scalar getEndVel(){
        return currentBin>=totalBins-1?endVel:velNodes[currentBin];
    }

    MotionProfile getInterpolator(){
        return interpolators.length==1?interpolators[0]:interpolators[currentBin];
    }

    @Override
    public Scalar getVel(double completion) {
        if(completion>getNextCompletionNode()&&currentBin!=totalBins-1){
            currentBin++;
            currentBinSize=getNextCompletionNode()-getLastCompletionNode();
            getInterpolator().init(getStartVel(),getEndVel(),totalDistance.multiply(currentBinSize));
        }
        return getInterpolator().getVel((completion-getLastCompletionNode())/currentBinSize);
    }
}
