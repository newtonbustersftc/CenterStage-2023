package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(name="Newton Autonomous", group="Main")
public class AutonomousGeneric extends LinearOpMode {

    RobotHardware robotHardware;
    RobotProfile robotProfile;

    ArrayList<RobotControl> taskList;

    long loopCount = 0;
    int countTasks = 0;
    private int delay;
    boolean isRedAlliance = false;

    Pose2d startPos = new Pose2d();

    public void initRobot() {
        try {
            robotProfile = RobotProfile.loadFromFile();
        }
        catch (Exception e) {
            RobotLog.e("RobotProfile reading exception" + e);
        }

        Logger.init();
        Logger.logFile("OpMode - Newton Autonomous");

        RobotFactory.reset();

        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        Logger.logFile("Init completed");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        robotHardware.enableManualCaching(false);
        robotHardware.clearBulkCache();
//        robotHardware.initSetup(this);
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        long loopStart = System.currentTimeMillis();
        long loopCnt = 0;
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
        String startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
        if(startPosMode.startsWith("RED"))
            isRedAlliance = true;

        RobotCVProcessor robotCVProcessor = new RobotCVProcessor(robotHardware, robotProfile, isRedAlliance);
        robotCVProcessor.initWebCam("Webcam 2", true);
        robotCVProcessor.frameProcessor.setSaveImage(false);
        Thread.sleep(3000); //take time to process
        RobotCVProcessor.TEAM_PROP_POS team_prop_pos = robotCVProcessor.getRecognitionResult();


//        AprilTagRecognition aprilTagRecognition = new AprilTagRecognition(true,hardwareMap);
//        aprilTagRecognition.initAprilTag();

        AutonomousTaskBuilder builder = new AutonomousTaskBuilder(robotHardware, robotProfile,team_prop_pos);
//        robotHardware.resetImu();

        //        signalRecognition.startRecognition();
        while (!isStopRequested() && !isStarted()) {
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt%100==0) {
                telemetry.addData("Start Position", startPosMode);
                telemetry.addData("CurrPose", currPose);
                telemetry.addData("LoopTPS", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.addData("Team prop pos = ", team_prop_pos);
                telemetry.update();
            }
        }
//        Logger.logFile("Recognition Result:" + aprilTagSignalRecognition.getRecognitionResult());
//        aprilTagSignalRecognition.stopRecognition();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        robotHardware.resetDriveAndEncoders();
        taskList = builder.buildTaskList();
        TaskReporter.report(taskList);
        Logger.logFile("Task list items: " + taskList.size());
        Logger.flushToFile();
        if (taskList.size() > 0) {
            taskList.get(0).prepare();
        }
        robotHardware.setMotorStopBrake(true);
        robotHardware.enableManualCaching(true);
        robotHardware.clearBulkCache();
//        robotHardware.turnDownSignalBlocker();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && taskList.size()>0) {
            loopCount++;

            robotHardware.clearBulkCache();
            robotHardware.getLocalizer().update();

            if (taskList.size() > 0) {
                taskList.get(0).execute();

                if (taskList.get(0).isDone()) {
                    String status = "MainTaskComplete: " + taskList.get(0) + " Pose:" + robotHardware.getLocalizer().getPoseEstimate();
                    RobotLog.ii("RobotTask", status);
                    Logger.logFile(status);
                    Logger.flushToFile();

                    taskList.get(0).cleanUp();
                    taskList.remove(0);

                    countTasks++;
                    telemetry.update();

                    if (taskList.size() > 0) {
                        RobotControl task = taskList.get(0);
                        task.prepare();
                        RobotLog.ii("RobotTask", "TaskPrepare " + task);
                    }
                }
            }
        }
        // Regardless, open the clamp to save the servo
        try {
            Logger.logFile("Autonomous - Final Location:" + robotHardware.getLocalizer().getPoseEstimate());
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }
        robotHardware.stopAll();
        robotHardware.setMotorStopBrake(false);
    }

}
