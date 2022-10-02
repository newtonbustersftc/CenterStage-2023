package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

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
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;


@Config
@TeleOp(name="RobotVision", group="Test")
public class RobotVisionTest extends LinearOpMode {

    RobotHardware robotHardware;
    RobotProfile robotProfile;
    RobotVision robotVision;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;

    public static Scalar LOWER_BOUND = new Scalar(10, 10, 100);
    public static Scalar UPPER_BOUND = new Scalar(30, 200, 200);
    public static int MIN_AREA = 100;

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

        long loopStart = System.currentTimeMillis();
        long loopCnt = 0;
        robotVision.startWebcam("Webcam", new CVPipeline());
        while (!isStarted()) {
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt%100==0) {
                telemetry.addData("CurrPose", currPose);
                telemetry.addData("LoopTPS:", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.update();
            }
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
 class CVPipeline extends OpenCvPipeline {

    Mat hsvMat = new Mat();
    Mat maskMat = new Mat();
    Mat hierarchey = new Mat();

    static Scalar DRAW_COLOR_RED = new Scalar(255, 0, 0);


     @Override
    public Mat processFrame(Mat input) {
        // 1. Convert to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        // 2. Create MASK
        if (RobotVisionTest.LOWER_BOUND.val[0] > RobotVisionTest.UPPER_BOUND.val[0]) {
             // RED situation
             Mat maskMat1 = new Mat();
             Mat maskMat2 = new Mat();
             Core.inRange(hsvMat, RobotVisionTest.LOWER_BOUND,
                     new Scalar(255, RobotVisionTest.UPPER_BOUND.val[1], RobotVisionTest.UPPER_BOUND.val[2]), maskMat1);
             Core.inRange(hsvMat, new Scalar(0, RobotVisionTest.LOWER_BOUND.val[1], RobotVisionTest.LOWER_BOUND.val[2]),
                     RobotVisionTest.UPPER_BOUND, maskMat2);
             Core.add(maskMat1, maskMat2, maskMat);
             maskMat1.release();
             maskMat2.release();
         } else {
             // Non RED situation
            Core.inRange(hsvMat, RobotVisionTest.LOWER_BOUND, RobotVisionTest.UPPER_BOUND, maskMat);
         }

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(maskMat, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > RobotVisionTest.MIN_AREA) {
                Rect rec = Imgproc.boundingRect(wrapper);
                //Rect drawRec = new Rect(rec.x*DIM_MULTIPLIER, rec.y*DIM_MULTIPLIER, rec.width*DIM_MULTIPLIER, rec.height*DIM_MULTIPLIER);
                Imgproc.rectangle(input, rec, DRAW_COLOR_RED, 2);
            }
        }
//        if (saveImage) {
//            //need to save pic to file
//            String timestamp = new SimpleDateFormat("MMdd-HHmmss", Locale.US).format(new Date());
//            Mat mbgr = new Mat();
//            Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
//            Imgcodecs.imwrite("/sdcard/FIRST/S" + timestamp + ".jpg", mbgr);
//            mbgr.release();
//        }
        return input;
    }
 }
