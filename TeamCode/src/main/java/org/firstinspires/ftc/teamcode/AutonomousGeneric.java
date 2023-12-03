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
import org.firstinspires.ftc.vision.VisionPortal;

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
    AprilTagRecognition aprilTagRecognition;

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
        if (startPosMode.startsWith("RED"))
            isRedAlliance = true;
        String parking = prefs.getString(AutonomousOptions.PARKING_PREF, AutonomousOptions.PARKING_LOCATION[0]);

        RobotCVProcessor robotCVProcessor = new RobotCVProcessor(robotHardware, robotProfile, isRedAlliance);
        robotCVProcessor.initWebCam("Webcam 2", true);
        Logger.logFile("1) Webcam2 status:"+robotCVProcessor.visionPortal.getCameraState());
        robotCVProcessor.frameProcessor.setSaveImage(true);
        Thread.sleep(1000); //take time to process

        Pose2d startingPose = robotProfile.getProfilePose("START_POSE_" + startPosMode);

        robotHardware.getLocalizer().setPoseEstimate(startingPose);
        while (!isStopRequested() && !isStarted()) {
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            robotCVProcessor.visionPortal.resumeStreaming();
            loopCnt++;
            if (loopCnt % 100 == 0) {
                telemetry.addData("Start Position", startPosMode);
                telemetry.addData("Start Pose2d", startingPose);
                telemetry.addData("CurrPose", currPose);
                telemetry.addData("Parking = ", parking);
                telemetry.addData("LoopTPS", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.addData("Team prop pos = ", robotCVProcessor.getRecognitionResult());
                telemetry.addData("2) Webcam2 status = ", robotCVProcessor.visionPortal.getCameraState());
                telemetry.update();
            }
        }
        RobotCVProcessor.TEAM_PROP_POS teamPropPos = robotCVProcessor.getRecognitionResult();
        if (teamPropPos==RobotCVProcessor.TEAM_PROP_POS.NONE) {
            Logger.logFile("Team prop recognition NONE, default to CENTER");
            teamPropPos = RobotCVProcessor.TEAM_PROP_POS.CENTER;
        }
        robotCVProcessor.close();
        Logger.logFile("3) Webcam2 status = "+ robotCVProcessor.visionPortal.getCameraState());
        aprilTagRecognition = new AprilTagRecognition(true, hardwareMap);
        aprilTagRecognition.initAprilTag();
        Logger.logFile("Webcam 1 status = "+ aprilTagRecognition.visionPortal.getCameraState());
        AutonomousTaskBuilder builder = new AutonomousTaskBuilder(robotHardware, robotProfile, teamPropPos, startingPose, aprilTagRecognition);

        robotHardware.resetDriveAndEncoders();
        robotHardware.getLocalizer().setPoseEstimate(startingPose);
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

        //test:

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && taskList.size() > 0) {
            loopCount++;

            robotHardware.clearBulkCache();
            robotHardware.getLocalizer().update();

            if (taskList.size() > 0) {
                taskList.get(0).execute();

                if (taskList.get(0).isDone()) {
                    String status = "MainTaskComplete: " + taskList.get(0) + " Pose:" + robotHardware.getLocalizer().getPoseEstimate()
                            + " gyro:" + robotHardware.getGyroHeading();
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
        } catch (Exception ex) {
        }
        robotHardware.stopAll();
        robotHardware.setMotorStopBrake(false);
        aprilTagRecognition.visionPortal.close();
    }
}
