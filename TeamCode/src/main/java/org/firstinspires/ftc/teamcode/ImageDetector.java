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

    public ImageDetector() {
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if(workingMatrix.empty()) {
            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matOneRing = workingMatrix.submat(167, 177, 0, 70);
        Mat matFourRings = workingMatrix.submat(147, 167, 0, 70);

        Imgproc.rectangle(workingMatrix, new Rect(0, 167, 70, 10), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(0, 147, 70, 20), new Scalar(0, 255, 0));

        oneRingTotal = Core.sumElems(matOneRing).val[2];
        fourRingsTotal = Core.sumElems(matFourRings).val[2];

       double thresholdOne = 62500, thresholdFour = 135000; //old thresholdFour = 130000
       if(fourRingsTotal < thresholdFour) count = "FOUR";
       else if(oneRingTotal < thresholdOne) count = "ONE";
       else count = "ZERO";
       /*if(oneRingTotal < thresholdOne)
       {
           if(fourRingsTotal < thresholdFour)
           {
               count = "FOUR";
           }
           else
               count = "ONE";
       } else count = "ZERO";*/

        return workingMatrix;
    }
}
