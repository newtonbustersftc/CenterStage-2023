package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

public class RobotCVProcessor {
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    HardwareMap hardwareMap;
    VisionPortal visionPortal;
    FrameProcessor frameProcessor;
    boolean isRed;

    public enum TEAM_PROP_POS { LEFT, CENTER, RIGHT, NONE };

    public RobotCVProcessor(RobotHardware robotHardware, RobotProfile robotProfile, boolean isRed) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.hardwareMap = robotHardware.getHardwareMap();
        this.isRed = isRed;
    }

    // it start streaming right away
    public void initWebCam(String deviceName, boolean withPreview) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        frameProcessor = new FrameProcessor(robotProfile, isRed);
        builder.setCamera(hardwareMap.get(WebcamName.class, deviceName));
        builder.addProcessor(frameProcessor);
        builder.setCameraResolution(new Size(640, 360));
        builder.enableLiveView(withPreview);      // Enable LiveView (RC preview).
        builder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.
        visionPortal = builder.build();
    }

    public float getFrameRate() {
        return visionPortal.getFps();
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void stopStreaming() {
        try {
            visionPortal.stopStreaming();
        }
        catch (Exception ex) {
            Logger.logFile("stopStreaming:" + ex);
        }
    }

    public void saveNextImage() {
        frameProcessor.setSaveImage(true);
    }

    public void close() {
        try {
            visionPortal.close();
        }
        catch (Exception ex) {
            Logger.logFile("vision close:" + ex);
        }
    }

    public TEAM_PROP_POS getRecognitionResult() {
        return frameProcessor.getRecognitionResult();
    }

    class FrameProcessor implements VisionProcessor {
        RobotProfile profile;
        boolean isRed;
        boolean saveImage = false;
        Mat hsvMat = new Mat();
        Mat maskMat = new Mat();
        Mat hierarchey = new Mat();
        Scalar DRAW_COLOR = new Scalar(0, 255, 0);
        Scalar lowerBound, upperBound;
        int lastCenter = -1;

        public FrameProcessor(RobotProfile profile, boolean isRed) {
            this.profile = profile;
            this.isRed = isRed;
            if (isRed) {
                lowerBound = this.profile.cvParam.redLowerBound;
                upperBound = this.profile.cvParam.redUpperBound;
            }
            else {
                lowerBound = this.profile.cvParam.blueLowerBound;
                upperBound = this.profile.cvParam.blueUpperBound;
            }
            Logger.logFile("isRed = "+ isRed);
            Logger.logFile("lowerBound="+lowerBound);
            Logger.logFile("upperBound="+upperBound);
        }

        public void setSaveImage(boolean saveImage) {
            this.saveImage = saveImage;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Sample code to high light RED objects
            // 1. Crop and Convert to HSV
            int offsetX = frame.width()*robotProfile.cvParam.cropLeftPercent/100;
            int offsetY = frame.height()*robotProfile.cvParam.cropTopPercent/100;
            Mat procMat = frame.submat(new Rect(offsetX, offsetY,
                    frame.width()*(100-robotProfile.cvParam.cropLeftPercent-robotProfile.cvParam.cropRightPercent)/100,
                    frame.height()*(100-robotProfile.cvParam.cropTopPercent-robotProfile.cvParam.cropBottomPercent)/100));
            Imgproc.cvtColor(procMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
            // 2. Create MASK
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
                Logger.logFile("here in blue: upper="+upperBound + " lower="+lowerBound);
                Mat maskMat1 = new Mat();
                Mat maskMat2 = new Mat();
//                Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
//                Core.inRange(hsvMat, lowerBound,
//                        new Scalar(260,100,100), maskMat1);
//                Core.inRange(hsvMat, new Scalar(200, 50, 50),
//                        upperBound, maskMat2);
//                Core.add(maskMat1, maskMat2, maskMat);
                maskMat1.release();
                maskMat2.release();
                Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
            }
            // 3. Loop through the contours, find the largest one
            double lastArea = 100;
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(maskMat, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Iterator<MatOfPoint> each = contours.iterator();
            Logger.logFile("size of contours: "+contours.size());
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                Logger.logFile("area="+area);
                if (area > 1000) {
                    Rect rec = Imgproc.boundingRect(wrapper);
                    Imgproc.rectangle(frame, new Rect(offsetX + rec.x,
                            offsetY + rec.y, rec.width, rec.height), DRAW_COLOR, 2);
                    if (area>lastArea) {
                        lastCenter = offsetX + rec.x + rec.width/2;
                        lastArea = area;
                        Logger.logFile("last center: "+lastCenter);
                        Logger.logFile("last area: "+ lastArea);if(saveImage){
                            saveImage(frame);
                        }
                    }
                }
            }

            Logger.logFile("so.... last area = "+lastArea);

            return frame;
        }

        private void saveImage(Mat frame) {
            //need to save pic to file
            String timestamp = new SimpleDateFormat("MMdd-HHmmss-S", Locale.US).format(new Date());
            Mat mbgr = new Mat();
            Imgproc.cvtColor(frame, mbgr, Imgproc.COLOR_RGB2BGR, 3);
            Imgcodecs.imwrite("/sdcard/FIRST/S" + timestamp + ".jpg", mbgr);
            mbgr.release();
            saveImage = false;
        }

        public TEAM_PROP_POS getRecognitionResult() {
            if (lastCenter == -1) {
                return TEAM_PROP_POS.NONE;
            }
            else if (lastCenter < 720/4) {
                return TEAM_PROP_POS.LEFT;
            }
            else if (lastCenter > 720*3/4) {
                return TEAM_PROP_POS.RIGHT;
            }
            return TEAM_PROP_POS.CENTER;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }
    }
}
