package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;


public class PoleRecognition {

    RobotVision rVision;
    RobotProfile rProfile;

    CVPipelinePole pipe;

    public PoleRecognition(RobotVision rVision, RobotProfile rProfile) {
        this.rVision = rVision;
        this.rProfile = rProfile;
        this.pipe = new CVPipelinePole(rProfile);
    }

    public void stopRecognition() {
        rVision.stopWebcam("Webcam 2");
    }

    public void startRecognition() {
        rVision.initWebCam("Webcam 2", false);
        rVision.startWebcam("Webcam 2", pipe);
    }

    public int getPoleCenterOnImg() {
        return pipe.getPoleCenter();
    }

    public int getPoleWidthOnImg() {
        return pipe.getPoleWidth();
    }
}

class CVPipelinePole extends OpenCvPipeline {

    RobotHardware robotHardware;

    int poleCenter, poleWidth;

    RobotProfile robotProfile;

    public CVPipelinePole(RobotProfile robotProfile) {
        this.robotProfile = robotProfile;
    }

    Mat hsvMat = new Mat();
    Mat hierarchey = new Mat();

    static Scalar DRAW_COLOR_RED = new Scalar(255, 0, 0);

    boolean saveImage = false;
    int offsetX, offsetY;

    @Override
    public Mat processFrame(Mat input) {
        Mat maskMat = new Mat();
        offsetX = 600;
        offsetY = 0;
        Mat procMat = input.submat(new Rect(offsetX, offsetY,
                40,
                input.height()));

        // 1. Convert to HSV
        Imgproc.cvtColor(procMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        // 2. Create MASK
        Scalar lowerBound = new Scalar(PoleSampleOpMode.MASK_LOWER_BOUND_H, PoleSampleOpMode.MASK_LOWER_BOUND_S, PoleSampleOpMode.MASK_LOWER_BOUND_V);
        Scalar upperBound = new Scalar(PoleSampleOpMode.MASK_UPPER_BOUND_H, PoleSampleOpMode.MASK_UPPER_BOUND_S, PoleSampleOpMode.MASK_UPPER_BOUND_V);

        Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(maskMat, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > PoleSampleOpMode.MIN_SIZE) {
                Rect rec = Imgproc.boundingRect(wrapper);
                Imgproc.rectangle(input, new Rect(rec.x + 600, rec.y, rec.width, rec.height), DRAW_COLOR_RED, 2);
                poleCenter = rec.y + rec.height/2;
                poleWidth = rec.height;

            }
        }
        maskMat.release();
        if (saveImage) {
            //need to save pic to file
            String timestamp = new SimpleDateFormat("MMdd-HHmmss", Locale.US).format(new Date());
            Mat mbgr = new Mat();
            Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
            Imgcodecs.imwrite("/sdcard/FIRST/S" + timestamp + ".jpg", mbgr);
            mbgr.release();
            saveImage = false;
        }
        return input;
    }

    public int getPoleCenter() {
        return poleCenter;
    }

    public int getPoleWidth() {
        return poleWidth;
    }

    public void saveNextImg() {
        Logger.logFile("Save image clicked");
        saveImage = true;
    }
}