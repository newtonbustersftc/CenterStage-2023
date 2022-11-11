package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

@Autonomous(name="PoleSampleTest", group="Test")
public class PoleSampleOpMode extends LinearOpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;
    OpenCvWebcam camera;
    boolean xPressed = false;
    public static int MASK_LOWER_BOUND_H = 20;
    public static int MASK_LOWER_BOUND_S = 70;
    public static int MASK_LOWER_BOUND_V = 70;
    public static int MASK_UPPER_BOUND_H = 45;
    public static int MASK_UPPER_BOUND_S = 255;
    public static int MASK_UPPER_BOUND_V = 255;
    public static int MIN_SIZE = 200;
    public static double LIGHT_POWER  = 0.1;

    public void initRobot() {
        try {
            robotProfile = RobotProfile.loadFromFile();
        }
        catch (Exception e) {
            RobotLog.e("RobotProfile reading exception" + e);
        }

        Logger.init();

        RobotFactory.reset();

        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        Logger.logFile("Init completed");
    }

    public boolean initWebCam(String deviceName, boolean withPreview) {
        if (withPreview) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, deviceName), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, deviceName));
        }
        final boolean[] doneOpen = {false};
        camera.openCameraDeviceAsync(

                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        RobotLog.i("Webcam " + deviceName + " opened");
                        Logger.logFile("Webcam " + deviceName + " opened");
                        doneOpen[0] = true;
                    }

                    @Override
                    public void onError(int errorCode) {
                        Logger.logFile("Failed to open Webcam: " + deviceName);
                        RobotLog.e("Failed to open Webcam: " + deviceName);
                        doneOpen[0] = true;
                    }
                }
        );
        while (!doneOpen[0]) {
            try {
                Thread.sleep(100);
            } catch (Exception ex) {
            }
        }
        try {
            Thread.sleep(100);
        } catch (Exception ex) {
        }
        return doneOpen[0];
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        robotHardware.resetTurretPos();
        initWebCam("Webcam 2", true);
        int g = camera.getGainControl().getGain();
        Logger.logFile("Current camera gain: " + g + " max gain:" + camera.getGainControl().getMaxGain());
        //camera.getGainControl().setGain(96);
        FocusControl fc = camera.getFocusControl();

        Logger.logFile("Camera focus min: " + fc.getMinFocusLength() + " max:" + fc.getMaxFocusLength() + " curr:" + fc.getFocusLength());

        CVPipelinePole pipeline = new CVPipelinePole(robotProfile);
        camera.setPipeline(pipeline);
        camera.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);

        int loopCnt = 0;
        long loopStart = System.currentTimeMillis();
        telemetry.setMsTransmissionInterval(250);
        while (!isStopRequested() && !isStarted()) {
            loopCnt++;
            telemetry.addData("Mode", "Pre-Active");
            telemetry.addData("X button", xPressed);
            if (loopCnt%100==0) {
                telemetry.addData("LoopTPS", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
            }
            telemetry.update();
            if (!xPressed && gamepad1.x) {
                for(int i = 0; i < 6; i++){
                    robotHardware.setTurretPosition(i * 25);
                    Thread.sleep(1000);
                    pipeline.saveNextImg();
                    Logger.logFile("Pipeline Center X " + i + ": " + pipeline.getCenterX());
                    Thread.sleep(500);
                }
                robotHardware.setTurretPosition(0);
                Thread.sleep(3000);
                for(int i = 1; i < 6; i++){
                    robotHardware.setTurretPosition(-i * 25);
                    Thread.sleep(1000);
                    pipeline.saveNextImg();
                    Logger.logFile("Pipeline Center X -" + i + ": " + pipeline.getCenterX());
                    Thread.sleep(500);
                }
                robotHardware.setTurretPosition(0);
            }
            xPressed = gamepad1.x;
        }
    }

}
class CVPipelinePole extends OpenCvPipeline {

    RobotHardware robotHardware;

    double poleCenter, poleWidth;

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
                Imgproc.rectangle(input, new Rect( rec.x + 600, rec.y, rec.width, rec.height), DRAW_COLOR_RED, 2);
                poleCenter = rec.y + 0.5 * rec.height;
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

    public double getCenterX(){
        return poleCenter;
    }

    public void saveNextImg() {
        Logger.logFile("Save image clicked");
        saveImage = true;
    }
}
