package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class ImageDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public static String count = "ZERO";
    public double oneRingTotal;
    public double fourRingsTotal;

    Mat matOneRing, matFourRings;

    public ImageDetector() {
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if(workingMatrix.empty()) {
            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        matOneRing = workingMatrix.submat(160, 170, 50, 100);
        matFourRings = workingMatrix.submat(140, 160, 50, 100);

        Imgproc.rectangle(workingMatrix, new Rect(50, 160, 50, 10), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(50, 140, 50, 20), new Scalar(0, 255, 0));


        oneRingTotal = Core.sumElems(matOneRing).val[2];
        fourRingsTotal = Core.sumElems(matFourRings).val[2];

       double thresholdOne = 42500, thresholdFour = 90000;
       if(oneRingTotal < thresholdOne)
       {
           if(fourRingsTotal < thresholdFour)
           {
               count = "FOUR";
           }
           else
               count = "ONE";
       } else count = "ZERO";

        return workingMatrix;
    }
}
