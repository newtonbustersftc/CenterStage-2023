package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal;
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
import java.util.concurrent.TimeUnit;

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
    CameraManagerInternal cameraManager;

//    CVPipeline pipeline = new CVPipeline();
//    int imgcnt = 0;

    public RobotVision(RobotHardware robotHardware, RobotProfile robotProfile) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.hardwareMap = robotHardware.getHardwareMap();
        this.cameraManager = (CameraManagerInternal) ClassFactory.getInstance().getCameraManager();
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
                    Logger.logFile("Failed to open Webcam: " + deviceName + " Error:" + errorCode);
                    RobotLog.e("Failed to open Webcam: " + deviceName + " Error:" + errorCode);
                    doneOpen = OpenState.FAILURE;
                }
            }
        );
        while (doneOpen == OpenState.WAIT) {
            try {
                Thread.sleep(100);
            }
            catch (Exception ex) {
            }
        }
        if (doneOpen==OpenState.FAILURE) {
            return false;
        }
        try {
            Thread.sleep(200);
        }
        catch (Exception ex) {
        }
        cameraMap.put(deviceName, camera);
        //int g = camera.getGainControl().getGain();
        //Logger.logFile("Current camera gain: " + g + " max gain:" + camera.getGainControl().getMaxGain());
        //camera.getGainControl().setGain(70);
        //camera.getExposureControl().setMode(ExposureControl.Mode.Auto);
        //camera.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.AUTO);
        FocusControl fc = camera.getFocusControl();
        Logger.logFile(deviceName + " Focus Fix mode support:" + fc.isModeSupported(FocusControl.Mode.Fixed));
        ExposureControl ec = camera.getExposureControl();
        Logger.logFile(deviceName + " Manual Exposure support:" + ec.isModeSupported(ExposureControl.Mode.Manual));
        GainControl gc = camera.getGainControl();
        Logger.logFile(deviceName + " Gain min:" + gc.getMinGain() + " max:" + gc.getMaxGain());
        PtzControl pc = camera.getPtzControl();
        if (pc!=null) {
            Logger.logFile(deviceName + " Ptz support min-zoom:" + pc.getMinZoom() + " max-zoom:" + pc.getMaxZoom());
        }
        else {
            Logger.logFile(deviceName + " no Ptz Control");
        }
        return true;
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
            if(pipeline instanceof CVTestPipeline) {  // RobotVisionTest
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//                FocusControl fc = webcam.getFocusControl();
//                Logger.logFile("Pole Cam focus min: " + fc.getMinFocusLength() + " max:" + fc.getMaxFocusLength() + " curr:" + fc.getFocusLength());
//                fc.setMode(FocusControl.Mode.Fixed);
//                ExposureControl ec = webcam.getExposureControl();
//                Logger.logFile("Exposure supported:" + ec.isExposureSupported());
//                Logger.logFile("Exposure Manual Mode Supported:" + ec.isModeSupported(ExposureControl.Mode.Manual));
//                Logger.logFile("Exposure AP Mode Supported:" + ec.isModeSupported(ExposureControl.Mode.AperturePriority));
//                Logger.logFile("Exposure SP Mode Supported:" + ec.isModeSupported(ExposureControl.Mode.ShutterPriority));
//                Logger.logFile("Exposure Manual Mode Supported:" + ec.isModeSupported(ExposureControl.Mode.Manual));
//                ec.setMode(ExposureControl.Mode.Manual);
//                Logger.logFile("Manual EC min:" + ec.getMinExposure(TimeUnit.MILLISECONDS) + " max:" + ec.getMaxExposure(TimeUnit.MILLISECONDS)+
//                        " curr:" + ec.getExposure(TimeUnit.MILLISECONDS));
//                ec.setExposure(20, TimeUnit.MILLISECONDS);
//
//                fc.setFocusLength(15);
//                GainControl gc = webcam.getGainControl();
//                Logger.logFile("Pole Cam Gain min:" + gc.getMinGain() + " max:" + gc.getMaxGain() + " curr:" + gc.getGain());
//                gc.setGain(64);
//
            }else if(pipeline instanceof AprilTagDetectionPipeline){
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            } else if (pipeline instanceof CVPipelinePole) {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FocusControl fc = webcam.getFocusControl();
                Logger.logFile("Pole Cam focus min: " + fc.getMinFocusLength() + " max:" + fc.getMaxFocusLength() + " curr:" + fc.getFocusLength());
                fc.setMode(FocusControl.Mode.Fixed);
                fc.setFocusLength(15);
            }
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

    public void setExposureMS(String deviceName, int timeMs) {
        OpenCvWebcam webcam = cameraMap.get(deviceName);
        if (webcam!=null) {
            ExposureControl ec = webcam.getExposureControl();
            ec.setMode(ExposureControl.Mode.Manual);
            ec.setExposure(timeMs, TimeUnit.MILLISECONDS);
        }
    }

    public void setGain(String deviceName, int gain) {
        OpenCvWebcam webcam = cameraMap.get(deviceName);
        if (webcam != null) {
            GainControl gc = webcam.getGainControl();
            gc.setGain(gain);
        }
    }

    public void setManualFocusLength(String deviceName, int focus) {
        OpenCvWebcam webcam = cameraMap.get(deviceName);
        if (webcam != null) {
            FocusControl fc = webcam.getFocusControl();
            fc.setMode(FocusControl.Mode.Fixed);
            fc.setFocusLength(focus);
        }
    }

    public void setWhiteBalance(String deviceName, int temp) {
        OpenCvWebcam webcam = cameraMap.get(deviceName);
        if (webcam != null) {
            WhiteBalanceControl wb = webcam.getWhiteBalanceControl();
            wb.setMode(WhiteBalanceControl.Mode.MANUAL);
            wb.setWhiteBalanceTemperature(temp);
        }
    }
}

