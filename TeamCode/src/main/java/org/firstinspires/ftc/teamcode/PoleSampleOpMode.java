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
    PoleRecognition poleRecog;
    boolean xPressed = false;
    public static int MASK_LOWER_BOUND_H = 20;
    public static int MASK_LOWER_BOUND_S = 70;
    public static int MASK_LOWER_BOUND_V = 70;
    public static int MASK_UPPER_BOUND_H = 45;
    public static int MASK_UPPER_BOUND_S = 255;
    public static int MASK_UPPER_BOUND_V = 255;
    public static int MIN_SIZE = 200;

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
        poleRecog = new PoleRecognition(robotHardware.getRobotVision(), robotProfile);

        Logger.logFile("Init completed");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        robotHardware.resetTurretPos();
        poleRecog.startRecognition();


        int loopCnt = 0;
        long loopStart = System.currentTimeMillis();
        telemetry.setMsTransmissionInterval(250);
        telemetry.addData("Mode", "Pre-Active");
        telemetry.addData("X button", xPressed);
        double armExt = robotProfile.hardwareSpec.extensionInitPos;
        robotHardware.setExtensionPosition(armExt);
        boolean upPressed = false;
        while (!gamepad1.x) {
            if (!upPressed && gamepad1.dpad_up) {
                upPressed = true;
                armExt = armExt + 0.05;
                robotHardware.setExtensionPosition(armExt);
            }
            upPressed = gamepad1.dpad_up;
            if (loopCnt % 100 == 0) {
                telemetry.addData("Center:", poleRecog.getPoleCenterOnImg());
                telemetry.addData("Width:", poleRecog.getPoleWidthOnImg());
                telemetry.addData("Arm Extension:", armExt);
                telemetry.addData("LoopTPS", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
            }
            loopCnt++;
            telemetry.update();
        }
        telemetry.update();
        for(int i = 0; i < 6; i++){
            robotHardware.setTurretPosition(i * 25);
            Thread.sleep(1000);
            //pipeline.saveNextImg();
            Logger.logFile("Pipeline Center X " + i + ": " + poleRecog.getPoleCenterOnImg() +  " Width:" + poleRecog.getPoleWidthOnImg());
            Thread.sleep(500);
        }
        robotHardware.setTurretPosition(0);
        Thread.sleep(3000);
        for(int i = 1; i < 6; i++){
            robotHardware.setTurretPosition(-i * 25);
            Thread.sleep(1000);
            //pipeline.saveNextImg();
            Logger.logFile("Pipeline Center X " + i + ": " + poleRecog.getPoleCenterOnImg() +  " Width:" + poleRecog.getPoleWidthOnImg());
            Thread.sleep(500);
        }
        robotHardware.setTurretPosition(0);

        
        poleRecog.stopRecognition();
    }
}
