package org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.List;

public class FindBestSample {
    public static Sample findBestSample(List<ColorBlobLocatorProcessor> colorLocators, Pose3D relcam, CamFieldProfile CamProfile){

        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();

        for(ColorBlobLocatorProcessor colorLocator:colorLocators){
            blobs.addAll(colorLocator.getBlobs());
        }

        ColorBlobLocatorProcessor.Util.filterByArea(100, 20000, blobs);

        Sample ClosestSample=new Sample(blobs.get(0),relcam,CamProfile);

        for(ColorBlobLocatorProcessor.Blob b : blobs) {
            if (new Sample(b,relcam,CamProfile).ODistance<ClosestSample.ODistance){ClosestSample=new Sample(b,relcam,CamProfile);}
        }

        return ClosestSample;
    }
}
