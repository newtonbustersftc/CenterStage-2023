package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.io.PrintWriter;


@Config
@TeleOp(name="TurretTurning", group="Test")
public class TurretRotateProfiler extends LinearOpMode {
    public static int TOTAL_REC_CNT = 50000;
    RobotHardware robotHardware;
    DcMotorEx turrMotor;
    RobotProfile robotProfile;
    ArrayList<RobotControl> taskList;
    int loop = 0;

    class TurretRecord {
        long ts; double power; double velocity; int position;
        public TurretRecord(long ts, double power, double velocity, int position) {
            this.ts = ts;
            this.power = power;
            this.velocity = velocity;
            this.position = position;
        }

        public String toString() {
            return "" + ts + "," + power + "," + velocity + "," + position;
        }
    }

    TurretRecord[] recording = new TurretRecord[TOTAL_REC_CNT];

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
        robotHardware.initSetupNoAuto(this);
        robotSleep(2000);
        robotHardware.grabberClose();
        robotSleep(500);
        robotHardware.setLiftPosition(robotProfile.hardwareSpec.liftSafeRotate);
        //robotHardware.setExtensionPosition((robotProfile.hardwareSpec.extensionDriverMin+robotProfile.hardwareSpec.extensionFullOutPos)/2);
        //robotHardware.setExtensionPosition(robotProfile.hardwareSpec.extensionFullOutPos);
        robotHardware.setExtensionPosition(robotProfile.hardwareSpec.extensionDriverMin);
        robotSleep(2000);
        robotHardware.enableManualCaching(true);
        turrMotor = hardwareMap.get(DcMotorEx.class,"Turret Motor");
        turrMotor.setPower(0);
        turrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Logger.logFile("Main Task Loop started");
        long loopStart = System.currentTimeMillis();
        while (!isStopRequested() && !isStarted()) {
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loop++;
            if (loop%100==0) {
                telemetry.addData("LoopTPS", (loop * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.update();
            }
        }
        // TEST
        testNoEncoder();
        //testSetPosition();
        // WRITE RESULTS
        String timestamp = new SimpleDateFormat("yyyyMMdd-HHmm", Locale.US).format(new Date());
        try {
            PrintWriter pw = new PrintWriter(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/turdat_" + timestamp + ".csv"));
            pw.println("ts,power,velocity,position");
            for(int i=0; i<loop; i++){
                pw.println(recording[i]);
            }
            pw.flush();
            pw.close();
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }
        try {
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }
    }

    void testNoEncoder() {
        double power;
        int timegap = 20;  //millisecond

        loop = 0;
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && loop < TOTAL_REC_CNT) {
            long deltaTime = System.currentTimeMillis() - startTime;
            power = 1;
            //power = deltaTime/timegap * 0.01;
            //power = Math.min(power, 1.0);
            if (deltaTime>2500) {
                power = 0;      // cut off power to glide after 3 seconds
            }
            if (deltaTime>4500) {
                break;
            }
            turrMotor.setPower(power);
            robotHardware.clearBulkCache();
            recording[loop] = new TurretRecord(deltaTime, power, turrMotor.getVelocity(), turrMotor.getCurrentPosition());
            loop++;
        }
        Logger.logFile("testNoEncoder done with loop:" + loop);
    }

    void testSetPosition() {
        long startTime = System.currentTimeMillis();
        double power = 1;
        turrMotor.setTargetPosition(5460);
        turrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turrMotor.setPower(1.0);
        loop = 0;
        while (opModeIsActive() && loop < TOTAL_REC_CNT) {
            long deltaTime = System.currentTimeMillis() - startTime;
            robotHardware.clearBulkCache();
            recording[loop] = new TurretRecord(deltaTime, power, turrMotor.getVelocity(), turrMotor.getCurrentPosition());
            loop++;
            if (deltaTime>6000) {
                break;
            }
        }
        Logger.logFile("testSetPosition done with loop:" + loop);
    }


    void robotSleep(long ms) {
        try {
            Thread.sleep(2000);
        }
        catch (Exception ex) {
        }
    }
}
