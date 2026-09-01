package org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.PrebuiltMotionProfiles;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.NormalizedAngle;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.HeadingProfile;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.MotionProfile;

public class LUT_HP implements HeadingProfile {
    double[] completionNodes;
    NormalizedAngle[] angNodes;
    NormalizedAngle startAng;
    NormalizedAngle endAng;
    Scalar totalDistance;
    HeadingProfile[] interpolators; //The interpolation motion profile that is run between nodes. Entering an interpolators list of length 1 will reuse the same interpolator for all interpolations. Otherwise, the interpolators list should be 1 element longer than each of the nodes lists

    int totalBins;
    int currentBin;
    double currentBinSize;

    public LUT_HP(double[] completionNodes, NormalizedAngle[] angNodes, HeadingProfile[] interpolators){
        this.completionNodes=completionNodes;
        this.angNodes=angNodes;
        this.interpolators=interpolators;
        this.totalBins =completionNodes.length+1;
    }

    @Override
    public void init(NormalizedAngle startAng, NormalizedAngle endAng, Scalar totalDistance) {
        this.startAng=startAng;
        this.endAng=endAng;
        currentBin=0;
    }

    double getLastCompletionNode(){
        return currentBin==0?0:completionNodes[currentBin-1];
    }

    double getNextCompletionNode(){
        return currentBin>=totalBins-1?1:completionNodes[currentBin];
    }

    NormalizedAngle getStartAng(){
        return currentBin==0?startAng:angNodes[currentBin-1];
    }

    NormalizedAngle getEndAng(){
        return currentBin>=totalBins-1?startAng:angNodes[currentBin];
    }

    HeadingProfile getInterpolator(){
        return interpolators.length==1?interpolators[0]:interpolators[currentBin];
    }

    @Override
    public NormalizedAngle getAng(double completion) {
        if(completion>getNextCompletionNode()&&currentBin!=totalBins-1){
            currentBin++;
            currentBinSize=getNextCompletionNode()-getLastCompletionNode();
            getInterpolator().init(getStartAng(),getEndAng(),totalDistance.multiply(currentBinSize));
        }
        return getInterpolator().getAng((completion-getLastCompletionNode())/currentBinSize);
    }
}
