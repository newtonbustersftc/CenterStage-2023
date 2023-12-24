package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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
import java.util.concurrent.TimeUnit;

/** Copied from original AprilTagRecognition, clean up the code and add Pixel recognition
 */
public class PixelBoardVision {
    AprilTagProcessor aprilTag;
    PixelReader pixelReader;
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    String cameraName;

    VisionPortal visionPortal;

    public PixelBoardVision(RobotHardware robotHardware, RobotProfile robotProfile, String cameraName){
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.cameraName = cameraName;
    }

    public void init(){
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(robotHardware.getHardwareMap().get(WebcamName.class, cameraName));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        pixelReader = new PixelReader(robotHardware, robotProfile, aprilTag);
        builder.addProcessor(pixelReader);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
//        try {
//            setManualExposure(6, 250);
//        }
//        catch (Exception ex) {
//            Logger.logFile("Exception when initializing aprilTag " + ex);
//        }
        Logger.logFile("PixelBoard visionPortal camera state:"+visionPortal.getCameraState());
        Logger.logFile("visionPortal fps="+visionPortal.getFps());
    }

    public void saveNextImage() {
        pixelReader.setSaveImage(true);
    }

    public void stopStream(){
        visionPortal.stopStreaming();
    }

    public void resumeStream(){
        visionPortal.resumeStreaming();
    }

    public List getAprilTagResult(){
        return aprilTag.getDetections();
    }

    public Scalar getMeanLeft() {
        return pixelReader.meanLeft;
    }

    public Scalar getMeanRight() {
        return pixelReader.meanRight;
    }

    public boolean isLeftTaken() {
        if (pixelReader.meanLeft!=null) {
            return pixelReader.meanLeft.val[2]>robotProfile.cvParam.maxBlackValue;
        }
        return false;
    }

    public boolean isRightTaken() {
        if (pixelReader.meanRight!=null) {
            return pixelReader.meanRight.val[2]>robotProfile.cvParam.maxBlackValue;
        }
        return false;
    }

    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Logger.logFile("Camera" + "Waiting");
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                Thread.sleep(20);
            }

            Logger.logFile("Camera Ready");
            Logger.flushToFile();
        }

        // Set camera controls unless we are stopping.
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            Thread.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            Thread.sleep(20);
        }
    }

    class PixelReader implements VisionProcessor {
        RobotHardware robotHardware;
        RobotProfile profile;
        AprilTagProcessor aprilTagProcessor;
        boolean isRed;
        boolean saveImage = false;
        Scalar DRAW_COLOR = new Scalar(0, 255, 0);
        Scalar meanLeft = null, meanRight = null;

        public PixelReader(RobotHardware robotHardware, RobotProfile profile, AprilTagProcessor aprilTagProcessor) {
            this.robotHardware = robotHardware;
            this.profile = profile;
            this.aprilTagProcessor = aprilTagProcessor;
        }

        public void setSaveImage(boolean saveImage) {
            this.saveImage = saveImage;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            for(AprilTagDetection tag : aprilTag.getDetections()) {
                if (tag.metadata.id == robotHardware.desiredAprilTagId) {
                    Mat m1, m2, hsv1, hsv2;
                    int x1 = (int) (tag.center.x + (tag.corners[3].x - tag.corners[0].x));
                    int y1 = (int) (tag.center.y - (tag.corners[0].y - tag.corners[3].y));
                    int h = (int) ((tag.corners[0].y - tag.corners[3].y) * 5 / 4);
                    int w = (int) ((tag.corners[1].x - tag.corners[0].x) * 5 / 4);
                    int x0 = x1 + (int) ((tag.corners[3].x - tag.corners[0].x)  * 5 / 4);
                    int y0 = y1 - h;
                    m1 = frame.submat(new Rect(x0 - w, y0, w, h));
                    m2 = frame.submat(new Rect(x0, y0, w, h));
                    hsv1 = new Mat();
                    hsv2 = new Mat();
                    Imgproc.cvtColor(m1, hsv1, Imgproc.COLOR_RGB2HSV_FULL);
                    Imgproc.cvtColor(m2, hsv2, Imgproc.COLOR_RGB2HSV_FULL);
                    meanLeft = Core.mean(hsv1);
                    meanRight = Core.mean(hsv2);
                    hsv1.release();
                    hsv2.release();
                    Imgproc.rectangle(frame, new Rect(x0 - w, y0, w, h), DRAW_COLOR, 2);
                    Imgproc.rectangle(frame, new Rect(x0, y0, w, h), DRAW_COLOR, 2);

                }
            }
            if (saveImage) {
                //need to save pic to file
                String timestamp = new SimpleDateFormat("MMdd-HHmmss-S", Locale.US).format(new Date());
                Mat mbgr = new Mat();
                Imgproc.cvtColor(frame, mbgr, Imgproc.COLOR_RGB2BGR, 3);
                Imgcodecs.imwrite("/sdcard/FIRST/P" + timestamp + ".jpg", mbgr);
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
