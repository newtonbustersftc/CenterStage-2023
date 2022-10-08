package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


@Config
@TeleOp(name="Signal Recognition", group="Test")
public class SignalRecognition extends LinearOpMode {

    RobotHardware robotHardware;
    RobotProfile robotProfile;
    RobotVision robotVision;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;

    public static int L1 = 70;
    public static int L2 = 100;
    public static int L3 = 50;
    public static int H1 = 100;
    public static int H2 = 255;
    public static int H3 = 255;
    public static int MIN_AREA = 100;

    // GREEN: 60, 30, 60 -> 100, 255, 255
    // RED: 230, 60, 60 -> 15, 255, 255

    public void initRobot() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);

        Logger.logFile("Init completed");
    }

    @Override
    public void runOpMode() {

        initRobot();
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        robotVision = robotHardware.getRobotVision();
        robotVision.initWebCam("Webcam", true);

        CVPipelineSignal pipe = new CVPipelineSignal();
        robotVision.startWebcam("Webcam", pipe);
        while (!isStarted()) {
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            telemetry.addData("Sees Red?", pipe.getIsRed());
            telemetry.addData("Sees Green?", pipe.getIsGreen());
            telemetry.update();
        }

        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        taskList = new ArrayList<RobotControl>();

        taskList.add(new RobotSleep(1000));

        robotHardware.setMotorStopBrake(true);
        TaskReporter.report(taskList);
        Logger.logFile("Task list items: " + taskList.size());
        Logger.flushToFile();

        if (taskList.size()>0) {
            Logger.logFile("Task Prepare " + taskList.get(0));
            taskList.get(0).prepare();
        }
        // run until the end of the match (driver presses STOP)
        // run until the end of the match (driver presses STOP)
        long startTime = System.currentTimeMillis();
        int cnt = 100;
        double veloSum = 0;
        robotVision.startWebcam("Webcam", null);
        Logger.logFile("Main Task Loop started");

        while (opModeIsActive()) {
            loopCount++;
            robotHardware.clearBulkCache();
            robotHardware.getLocalizer().update();
            try {
                Logger.flushToFile();
            }
            catch (Exception ex) {
            }
            if (taskList.size() > 0) {
                taskList.get(0).execute();
                if (taskList.get(0).isDone()) {
                    Logger.logFile("MainTaskComplete: " + taskList.get(0) + " Pose:" + robotHardware.getLocalizer().getPoseEstimate());
                    taskList.get(0).cleanUp();
                    taskList.remove(0);
                    countTasks++;
                    telemetry.update();
                    if (taskList.size() > 0) {
                        taskList.get(0).prepare();
                    }
                }
            }
        }
        robotVision.stopWebcam("Webcam");
        try {
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }
    }
}
 class CVPipelineSignal extends OpenCvPipeline {

    Mat hsvMat = new Mat();
    Mat maskMat = new Mat();
    Mat hierarchey = new Mat();

    static Scalar DRAW_COLOR_RED = new Scalar(255, 0, 0);

    int offsetX, offsetY;

    boolean findRed = true;
    boolean findGreen = true;

     @Override
    public Mat processFrame(Mat input) {
        // 1. Convert to HSV
         int CROP_LEFT_PERCENT = 25;
         int CROP_RIGHT_PERCENT = 25;
         int CROP_TOP_PERCENT = 65;
         int CROP_BOTTOM_PERCENT = 15;
         offsetX = input.width()*CROP_LEFT_PERCENT/100;
         offsetY = input.height()*CROP_TOP_PERCENT/100;
         Mat procMat = input.submat(new Rect(offsetX, offsetY,
                 input.width()*(100-CROP_LEFT_PERCENT-CROP_RIGHT_PERCENT)/100,
                 input.height()*(100-CROP_TOP_PERCENT-CROP_BOTTOM_PERCENT)/100));

        Imgproc.cvtColor(procMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        // 2. Create MASK
         Scalar lowerBoundRed = new Scalar(230, 60, 60);
         Scalar upperBoundRed = new Scalar(15, 255, 255);
         Scalar lowerBoundGreen = new Scalar(60, 30, 60);
         Scalar upperBoundGreen = new Scalar(100, 255, 255);

         // GREEN: 60, 30, 60 -> 100, 255, 255
         // RED: 230, 60, 60 -> 15, 255, 255

         findRed = checkColor(lowerBoundRed, upperBoundRed, 100, hsvMat, input);
         findGreen = checkColor(lowerBoundGreen, upperBoundGreen, 100, hsvMat, input);

//        if (saveImage) {
//            //need to save pic to file
//            String timestamp = new SimpleDateFormat("MMdd-HHmmss", Locale.US).format(new Date());
//            Mat mbgr = new Mat();
//            Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
//            Imgcodecs.imwrite("/sdcard/FIRST/S" + timestamp + ".jpg", mbgr);
//            mbgr.release();
//        }

        Imgproc.rectangle(input, new Rect(offsetX, offsetY, procMat.width(), procMat.height()), DRAW_COLOR_RED, 2);
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
