package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

/**
 2020.11.05
 claire
 **/

public class RobotVision {
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    HardwareMap hardwareMap;
    public enum AutonomousGoal { NONE }
    enum OpenState { WAIT, SUCCESS, FAILURE}
    OpenState doneOpen;

//    CVPipeline pipeline = new CVPipeline();
//    int imgcnt = 0;

    public RobotVision(RobotHardware robotHardware, RobotProfile robotProfile) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.hardwareMap = robotHardware.getHardwareMap();
    }

    HashMap<String, OpenCvWebcam> cameraMap = new HashMap<String, OpenCvWebcam>();
    public boolean initWebCam(String deviceName, boolean withPreview) {
        OpenCvWebcam camera;
        if (withPreview) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, deviceName), cameraMonitorViewId);
        }
        else {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, deviceName));
        }
        cameraMap.put(deviceName, camera);
        doneOpen = OpenState.WAIT;
        camera.openCameraDeviceAsync(
            new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    RobotLog.i("Webcam " + deviceName + " opened");
                    Logger.logFile("Webcam " + deviceName + " opened");
                    doneOpen = OpenState.SUCCESS;
                }

                @Override
                public void onError(int errorCode) {
                    Logger.logFile("Failed to open Webcam: " + deviceName);
                    RobotLog.e("Failed to open Webcam: " + deviceName);
                    doneOpen = OpenState.FAILURE;
                }
            }
        );
        while (doneOpen==OpenState.WAIT) {
            try {
                Thread.sleep(100);
            }
            catch (Exception ex) {
            }
        }
        try {
            Thread.sleep(100);
        }
        catch (Exception ex) {
        }
        int g = camera.getGainControl().getGain();
        Logger.logFile("Current camera gain: " + g + " max gain:" + camera.getGainControl().getMaxGain());
        camera.getGainControl().setGain(50);
        return doneOpen==OpenState.SUCCESS;
    }

    class SavePicturePipeline extends OpenCvPipeline {
        String deviceName;
        boolean saveImage = false;

        public SavePicturePipeline(String deviceName) {
            this.deviceName = deviceName;
        }

        public void saveNextImage() {
            saveImage = true;
        }

        @Override
        public Mat processFrame(Mat input) {
            if (saveImage) {
                String timestamp = new SimpleDateFormat("MMdd-HHmmssSSS", Locale.US).format(new Date());
                Mat mbgr = new Mat();
                Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
                Imgcodecs.imwrite("/sdcard/FIRST/" + deviceName + "-" + timestamp + ".jpg", mbgr);
                mbgr.release();
                saveImage = false;
            }
            return input;
        }
    }

    public void startWebcam(String deviceName, OpenCvPipeline pipeline) {
        OpenCvWebcam webcam = cameraMap.get(deviceName);
        if (webcam!=null) {
            webcam.setPipeline(pipeline);
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }
        //rearCamera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public void stopWebcam(String deviceName) {
        OpenCvWebcam webcam = cameraMap.get(deviceName);
        if (webcam!=null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }
}

