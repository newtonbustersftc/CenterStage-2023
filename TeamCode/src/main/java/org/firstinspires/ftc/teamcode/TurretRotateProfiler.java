package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.geometry.euclidean.twod.Segment;
import org.firstinspires.ftc.teamcode.drive.opmode1.TurretMotionProfileGenerator;
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
    public int TOTAL_REC_CNT = 50000;
    RobotHardware robotHardware;
    DcMotorEx turrMotor;
    RobotProfile robotProfile;
    ArrayList<RobotControl> taskList;
    int loop = 0;

    class TurretRecord {
        double ts; double power; double velocity; int position; double degrees;
        public TurretRecord(double ts, double power, double velocity, int position, double degrees) {
            this.ts = ts;
            this.power = power;
            this.velocity = velocity;
            this.position = position;
            this.degrees = degrees;
        }

        public String toString() {
            return "" + ts + "," + power + "," + velocity + "," + position+","+degrees;
        }
    }

    class TurretRecord_mp {
        double ts; double power; double velocity; int position; double acceleration;
        public TurretRecord_mp(double ts, double power, double velocity, int position, double acceleration) {
            this.ts = ts;
            this.power = power;
            this.velocity = velocity;
            this.position = position;
            this.acceleration = acceleration;
        }

        public String toString() {
            return "" + ts + "," + power + "," + velocity + "," + position +","+acceleration;
        }
    }

    TurretRecord[] recording = new TurretRecord[TOTAL_REC_CNT];
    TurretRecord_mp[] recording_RR_profile = new TurretRecord_mp[TOTAL_REC_CNT];

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
        robotHardware.setLiftPosition(robotProfile.hardwareSpec.liftDropPos[5]);
        //robotHardware.setExtensionPosition((robotProfile.hardwareSpec.extensionDriverMin+robotProfile.hardwareSpec.extensionFullOutPos)/2);
        robotHardware.setExtensionPosition(robotProfile.hardwareSpec.extensionFullOutPos);
        //robotHardware.setExtensionPosition(robotProfile.hardwareSpec.extensionDriverMin);
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
        //testNoEncoder();
        //testSetPosition();
        test_RR_MotionProfile();

        // WRITE RESULTS
        String timestamp = new SimpleDateFormat("yyyyMMdd-HHmm", Locale.US).format(new Date());
        try {
            PrintWriter pw = new PrintWriter(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/turdat_" + timestamp + ".csv"));
            pw.println("ts,power,velocity,position,degree");
            if(recording.length>0) {
                for (int i = 0; i < loop; i++) {
                    pw.println(recording[i]);
                }
            }
            pw.flush();
            pw.close();

            //test, print the second set of data from motion state
            PrintWriter pw_mp = new PrintWriter(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/turdat_RR" + timestamp + ".csv"));
            pw_mp.println("ts,power,velocity,position");
            if(recording_RR_profile.length>0) {
                for (int i = 0; i < loop; i++) {
                    pw_mp.println(recording_RR_profile[i]);
                }
            }
            pw_mp.flush();
            pw_mp.close();
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
        power = 1;
        double power_down = 0.0;
        turrMotor.setPower(power);
        int RAM_UP_TIME = -1;      //ms
        int RAM_DOWN_START = 450;   //ms
        int RAM_DOWN_TIME = 950;    //ms
        double pw_now;
        while (opModeIsActive() && loop < TOTAL_REC_CNT) {
            long deltaTime = System.currentTimeMillis() - startTime;
            if (deltaTime < RAM_UP_TIME) {
                pw_now = (deltaTime * power) / RAM_UP_TIME;
            }
            else if (deltaTime >= RAM_UP_TIME && deltaTime <RAM_DOWN_START) {
                pw_now = power;
            }
            else if (deltaTime >= RAM_DOWN_START && deltaTime < (RAM_DOWN_START + RAM_DOWN_TIME)) {
                pw_now = (power - power_down)*(RAM_DOWN_START + RAM_DOWN_TIME - deltaTime) / RAM_DOWN_TIME + power_down;
            }
            else {
                pw_now = power_down;
            }
            //power = deltaTime/timegap
            // * 0.01;
            //power = Math.min(power, 1.0);
            turrMotor.setPower(pw_now);

            if (deltaTime>6000) {
                break;
            }
            robotHardware.clearBulkCache();
            recording[loop] = new TurretRecord(deltaTime, pw_now, turrMotor.getVelocity(), turrMotor.getCurrentPosition(), 0);
            loop++;
        }
        Logger.logFile("testNoEncoder done with loop:" + loop);
    }

    void testSetPosition() {
        long startTime = System.currentTimeMillis();
        double power = 1;
        turrMotor.setTargetPosition(2200);
        turrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turrMotor.setPower(1.0);
        loop = 0;
        while (opModeIsActive() && loop < TOTAL_REC_CNT) {
            long deltaTime = System.currentTimeMillis() - startTime;
            robotHardware.clearBulkCache();
            recording[loop] = new TurretRecord(deltaTime, power, turrMotor.getVelocity(), turrMotor.getCurrentPosition(),0);
            loop++;
            if (deltaTime>6000) {
                break;
            }
        }
        Logger.logFile("testSetPosition done with loop:" + loop);
    }

    private void test_RR_MotionProfile(){
        int startFrom = 0;//42=>166,50=>187. 30=>119
        int endAt = 60; //60 inches => 205 degrees, 84 inches => 360 degrees 105 inches => 360 + 90 =450 degrees
        //above results:4.54x+32 or 0.015x^2 +2.49x + 29

        ArrayList segments = new ArrayList<Segment>();
        ArrayList segmentDt = new ArrayList<Double>();

        MotionProfile activeProfile = generateTurretProfile(startFrom, endAt);
        Logger.logFile("*******test_RR_MotionProfile**********");
        for(int i=0; i<activeProfile.getSegments().size(); i++){
            MotionSegment seg= activeProfile.getSegments().get(i);
            segments.add(seg);
            segmentDt.add(seg.getDt());
            Logger.logFile("seg dt="+seg.getDt());
            Logger.logFile("seg start X, V, A:" +seg.getStart().getX() +", " +
                    seg.getStart().getV() +", " +
                    seg.getStart().getA());
            Logger.logFile("seg end X, V, A"+seg.end().getX()+", " +
                    seg.getStart().getV() +", " +
                    seg.getStart().getA());

        }

        NanoClock clock = NanoClock.system();
        double newProfileStart = clock.seconds();
        MotionState motionState = null;
        Logger.logFile("activeProfile: duration="+activeProfile.duration());
        double targetPower=0;
        double delTime;
        double degrees=0;

        for(int i=0; i<segments.size(); i++ ){
           MotionSegment segment = (MotionSegment) segments.get(i);
            Logger.logFile("segment="+ i + ": time="+segmentDt + ", x,v,a,j=" + segment.toString());
        }

        while (opModeIsActive() && loop < TOTAL_REC_CNT &&
                (clock.seconds()-newProfileStart)<= activeProfile.duration()) {
            delTime = clock.seconds() - newProfileStart;
            motionState = activeProfile.get(delTime);
            Logger.logFile("delTime = "+delTime);
            Logger.logFile("motion state:"+motionState.toString());
            targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);
            Logger.logFile("target power = " + targetPower);

            robotHardware.turretMotor.setPower(targetPower);
            Logger.logFile("deltaTime="+ delTime + ", X="+ motionState.getX() + ", targetPower" + robotHardware.turretMotor.getPower() +", motionState.v="+ motionState.getV() + ", motionState.a="+motionState.getA());
            robotHardware.clearBulkCache();
            degrees = ((double)turrMotor.getCurrentPosition())/(925*4) * 360;
            recording[loop] = new TurretRecord(delTime, turrMotor.getPower(), turrMotor.getVelocity(), turrMotor.getCurrentPosition(),degrees);
            recording_RR_profile[loop] = new TurretRecord_mp(delTime, targetPower, motionState.getV(), (int)motionState.getX(), motionState.getA());
            loop++;
//            if (deltaTime>6000) {
//                break;
//            }
        }
        Logger.logFile("final degrees = "+degrees);
        Logger.logFile("test_RR_MotionProfile done with loop:" + loop);
    }

    private MotionProfile generateTurretProfile(double startTurretPos, double goalTurretPos) {
        MotionState start = new MotionState(startTurretPos, 0, 0, 0);
        MotionState goal = new MotionState( goalTurretPos, 0,0, 0);
        return new TurretMotionProfileGenerator().generateSimpleMotionProfile(start, goal, 25.0, 25.0);
    }

    void robotSleep(long ms) {
        try {
            Thread.sleep(2000);
        }
        catch (Exception ex) {
        }
    }
}
