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
    public static String CAMERA_NAME = "Webcam 2";

    public PoleRecognition(RobotVision rVision, RobotProfile rProfile) {
        this.rVision = rVision;
        this.rProfile = rProfile;
        this.pipe = new CVPipelinePole(rProfile);
    }

    public void stopRecognition() {
        rVision.stopWebcam(CAMERA_NAME);
    }

    public void startRecognition() {
        rVision.initWebCam(CAMERA_NAME, true);
        rVision.setGain(CAMERA_NAME, rProfile.poleParameter.gain);
        rVision.setExposureMS(CAMERA_NAME, rProfile.poleParameter.exposureMs);
        rVision.setManualFocusLength(CAMERA_NAME, rProfile.poleParameter.focus);
        rVision.setWhiteBalance(CAMERA_NAME, rProfile.poleParameter.whiteBalance);
        rVision.startWebcam(CAMERA_NAME, pipe);
    }

    public void saveNextImg() {
        pipe.saveNextImg();
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
        Mat procMat = input.submat(new Rect(robotProfile.poleParameter.cropXywh[0],
                robotProfile.poleParameter.cropXywh[1],
                robotProfile.poleParameter.cropXywh[2],
                robotProfile.poleParameter.cropXywh[3]));

        // 1. Convert to HSV
        Imgproc.cvtColor(procMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        // 2. Create MASK
        Scalar lowerBound = robotProfile.poleParameter.lowerBound;
        Scalar upperBound = robotProfile.poleParameter.upperBound;

        Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(maskMat, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Iterator<MatOfPoint> each = contours.iterator();
        double largest = 0;
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > robotProfile.poleParameter.minSize && area > largest) {
                largest = area;
                Rect rec = Imgproc.boundingRect(wrapper);
                Imgproc.rectangle(input, new Rect(robotProfile.poleParameter.cropXywh[0]+ rec.x,
                        robotProfile.poleParameter.cropXywh[1] + rec.y, rec.width, rec.height), DRAW_COLOR_RED, 2);
                poleCenter = rec.x + rec.width/2;
                poleWidth = rec.width;

            }
        }
        maskMat.release();
        if (saveImage) {
            //need to save pic to file
            String timestamp = new SimpleDateFormat("MMdd-HHmmss-S", Locale.US).format(new Date());
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