package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


public class SignalRecognition {

    RobotVision rVision;
    RobotProfile rProfile;

    CVPipelineSignal pipe;

    public SignalRecognition(RobotVision rVision, RobotProfile rProfile) {
        this.rVision = rVision;
        this.rProfile = rProfile;
        this.pipe = new CVPipelineSignal(rProfile);
    }

    public void stopRecognition() {
        rVision.stopWebcam("Webcam 1");
    }

    public void startRecognition() {
        rVision.initWebCam("Webcam 1", true);
        try {
            Thread.sleep(2000);
        }
        catch (Exception e) {}
        rVision.startWebcam("Webcam 1", pipe);
    }

    public enum Result {POSITION1, POSITION2, POSITION3}
    public Result getRecognitionResult() {
        if (pipe.getIsRed()) {
            return Result.POSITION1;
        } else if (pipe.getIsGreen()) {
            return Result.POSITION2;
        } else {
            return Result.POSITION3;
        }
    }
}
 class CVPipelineSignal extends OpenCvPipeline {

    RobotProfile robotProfile;

    public CVPipelineSignal(RobotProfile robotProfile) {
        this.robotProfile = robotProfile;
    }

    Mat hsvMat = new Mat();
    Mat hierarchey = new Mat();

    static Scalar DRAW_COLOR_RED = new Scalar(255, 0, 0);

    int offsetX, offsetY;

    boolean findRed = true;
    boolean findGreen = true;

     @Override
    public Mat processFrame(Mat input) {
        // 1. Convert to HSV
         offsetX = input.width()*robotProfile.cvParam.cropLeftPercent/100;
         offsetY = input.height()*robotProfile.cvParam.cropTopPercent/100;
         Mat procMat = input.submat(new Rect(offsetX, offsetY,
                 input.width()*(100-robotProfile.cvParam.cropLeftPercent-robotProfile.cvParam.cropRightPercent)/100,
                 input.height()*(100-robotProfile.cvParam.cropTopPercent-robotProfile.cvParam.cropBottomPercent)/100));

        Imgproc.cvtColor(procMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        // 2. Create MASK
         Scalar lowerBoundRed = robotProfile.cvParam.redLowerBound;
         Scalar upperBoundRed = robotProfile.cvParam.redUpperBound;
         Scalar lowerBoundGreen = robotProfile.cvParam.greenLowerBound;
         Scalar upperBoundGreen = robotProfile.cvParam.greenUpperBound;

         findGreen = checkColor(lowerBoundGreen, upperBoundGreen, robotProfile.cvParam.minArea, hsvMat, input);
         findRed = checkColor(lowerBoundRed, upperBoundRed, robotProfile.cvParam.minArea, hsvMat, input);

//        if (saveImage) {
//            //need to save pic to file
//            String timestamp = new SimpleDateFormat("MMdd-HHmmss", Locale.US).format(new Date());
//            Mat mbgr = new Mat();
//            Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
//            Imgcodecs.imwrite("/sdcard/FIRST/S" + timestamp + ".jpg", mbgr);
//            mbgr.release();
//        }

        Imgproc.rectangle(input, new Rect(offsetX, offsetY, procMat.width(), procMat.height()), DRAW_COLOR_RED, 1);
        return input;
    }
    boolean checkColor(Scalar lowerBound, Scalar upperBound, int minSize, Mat hsvMat, Mat input) {
        Mat maskMat = new Mat();
        boolean result = false;
        if (lowerBound.val[0] > upperBound.val[0]) {
            // RED situation
            Mat maskMat1 = new Mat();
            Mat maskMat2 = new Mat();
            Core.inRange(hsvMat, lowerBound,
                    new Scalar(255, upperBound.val[1], upperBound.val[2]), maskMat1);
            Core.inRange(hsvMat, new Scalar(0, lowerBound.val[1], lowerBound.val[2]),
                    upperBound, maskMat2);
            Core.add(maskMat1, maskMat2, maskMat);
            maskMat1.release();
            maskMat2.release();
        } else {
            // Non RED situation
            Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
        }
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(maskMat, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > minSize) {
                Rect rec = Imgproc.boundingRect(wrapper);
                //Rect drawRec = new Rect(rec.x*DIM_MULTIPLIER, rec.y*DIM_MULTIPLIER, rec.width*DIM_MULTIPLIER, rec.height*DIM_MULTIPLIER);
                Imgproc.rectangle(input, new Rect(offsetX + rec.x, offsetY + rec.y, rec.width, rec.height), DRAW_COLOR_RED, 2);
                result = true;
            }
        }
        return result;
    }
    public boolean getIsRed() {
         return findRed;
    }
    public boolean getIsGreen() {
         return findGreen;
     }
 }
