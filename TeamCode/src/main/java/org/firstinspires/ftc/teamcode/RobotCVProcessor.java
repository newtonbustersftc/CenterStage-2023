package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
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

    public RobotCVProcessor(RobotHardware robotHardware, RobotProfile robotProfile) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.hardwareMap = robotHardware.getHardwareMap();
    }

    // it start streaming right away
    public void initWebCam(String deviceName, boolean withPreview) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        frameProcessor = new FrameProcessor();
        builder.setCamera(hardwareMap.get(WebcamName.class, deviceName));
        builder.addProcessor(frameProcessor);
        builder.setCameraResolution(new Size(640, 480));
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
        visionPortal.stopStreaming();
    }

    public void saveNextImage() {
        frameProcessor.setSaveImage(true);
    }

    public void close() {
        visionPortal.close();
    }

    class FrameProcessor implements VisionProcessor {
        boolean saveImage = false;
        Mat hsvMat = new Mat();
        Mat maskMat = new Mat();
        Mat hierarchey = new Mat();
        Scalar DRAW_COLOR = new Scalar(0, 255, 0);

        public void setSaveImage(boolean saveImage) {
            this.saveImage = saveImage;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Sample code to high light RED objects
            // 1. Convert to HSV
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
            // 2. Create MASK
            Scalar lowerBound = new Scalar(0, 60, 100);
            Scalar upperBound = new Scalar(15, 255, 255);

            Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(maskMat, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Iterator<MatOfPoint> each = contours.iterator();
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                if (area > 100) {
                    Rect rec = Imgproc.boundingRect(wrapper);
                    Imgproc.rectangle(frame, new Rect(rec.x,
                            rec.y, rec.width, rec.height), DRAW_COLOR, 2);
                }
            }
            if (saveImage) {
                //need to save pic to file
                String timestamp = new SimpleDateFormat("MMdd-HHmmss-S", Locale.US).format(new Date());
                Mat mbgr = new Mat();
                Imgproc.cvtColor(frame, mbgr, Imgproc.COLOR_RGB2BGR, 3);
                Imgcodecs.imwrite("/sdcard/FIRST/S" + timestamp + ".jpg", mbgr);
                mbgr.release();
                saveImage = false;
            }
            return frame;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }
    }
}
