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
        robotCVProcessor.frameProcessor.setSaveImage(false);
        Thread.sleep(3000); //take time to process
        RobotCVProcessor.TEAM_PROP_POS team_prop_pos = robotCVProcessor.getRecognitionResult();
        robotCVProcessor.close();

        aprilTagRecognition = new AprilTagRecognition(true, hardwareMap);
        aprilTagRecognition.initAprilTag();

        Pose2d starting_pose_blue_left = robotProfile.getProfilePose("START_POSE_BLUE_LEFT");
        Pose2d starting_pose_blue_right = robotProfile.getProfilePose("START_POSE_BLUE_RIGHT");
        Pose2d starting_pose_red_left = robotProfile.getProfilePose("START_POSE_RED_LEFT");
        Pose2d starting_pose_red_right = robotProfile.getProfilePose("START_POSE_RED_RIGHT");

        Pose2d startingPose = startPosMode.startsWith("BLUE") ? (startPosMode.contains("LEFT") ? starting_pose_blue_left : starting_pose_blue_right) :
                (startPosMode.contains("LEFT") ? starting_pose_red_left : starting_pose_red_right);

        AutonomousTaskBuilder builder = new AutonomousTaskBuilder(robotHardware, robotProfile, team_prop_pos, startingPose, aprilTagRecognition);
//        robotHardware.resetImu();
//        if (aprilTagRecognition.visionPortal.getCameraState().equals(VisionPortal.CameraState.CAMERA_DEVICE_READY)) {
//            aprilTagRecognition.stopStream();
//        }

        if (startPosMode.equals("BLUE_LEFT")) {
            robotHardware.getLocalizer().setPoseEstimate(starting_pose_blue_left);
        } else if (startPosMode.equals("BLUE_RIGHT")) {
            robotHardware.getLocalizer().setPoseEstimate(starting_pose_blue_right);
        } else if (startPosMode.equals("RED_LEFT")) {
            robotHardware.getLocalizer().setPoseEstimate(starting_pose_red_left);
        } else if (startPosMode.equals("RED_RIGHT")) {
            robotHardware.getLocalizer().setPoseEstimate(starting_pose_red_right);
        } else {
            // TODO: 11/14/23 ??
        }

        while (!isStopRequested() && !isStarted()) {
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt % 100 == 0) {
                telemetry.addData("Start Position", startingPose);
                telemetry.addData("CurrPose", currPose);
                telemetry.addData("Parking = ", parking);
                telemetry.addData("LoopTPS", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.addData("Team prop pos = ", team_prop_pos);
                telemetry.update();
            }
        }

//        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
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
        while (opModeIsActive() && taskList.size() > 0) {
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
            } catch (Exception ex) {
            }
            robotHardware.stopAll();
            robotHardware.setMotorStopBrake(false);
            aprilTagRecognition.visionPortal.close();
        }
}
