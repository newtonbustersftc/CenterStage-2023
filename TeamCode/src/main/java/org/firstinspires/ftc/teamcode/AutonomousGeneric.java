package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

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
    DriverOptions driverOptions;
//    RobotVision robotVision;

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

        RobotFactory.reset();

        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
//        robotHardware = new RobotHardware();
//        robotHardware.init(hardwareMap, robotProfile);

//        robotHardware.setMotorStopBrake(true);

        robotHardware.clearBulkCache();

        driverOptions = new DriverOptions();
        Logger.logFile("Init completed");
        try {
            SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
            String delayString = prefs.getString("start_delay", "0").replace(" sec", "");
            driverOptions.setStartDelay(Integer.parseInt(delayString));
            Logger.logFile("start_delay: " + driverOptions.getStartDelay());
            String freightDeliveryCount = prefs.getString("freight delivery count", "0").replace(" sec", "");
            driverOptions.setFreightDeliveryCount(Integer.parseInt(freightDeliveryCount));

            driverOptions.setStartingPositionModes(prefs.getString("starting position", ""));
            driverOptions.setParking(prefs.getString("parking", ""));
            String delay_parking_by_storage = prefs.getString("delay parking by storage", "0").replace( " sec", "");
            String delay_parking_by_warehouse = prefs.getString("delay parking by warehouse", "0").replace( " sec", "");
            driverOptions.setDelayParkingByStorageFromEnding(Integer.parseInt(delay_parking_by_storage));
            driverOptions.setDelayParkingByWarehouseFromEnding(Integer.parseInt(delay_parking_by_warehouse));
            driverOptions.setParkingOnly(prefs.getString("park only", ""));

            String isDuckBySideRoute = prefs.getString("Duck deliver to hub by side route", "");
            String duckParking = prefs.getString("duck parking direction", "");
            if(driverOptions.getStartingPositionModes().contains("RED")){
                driverOptions.setDuckParkingDirection(duckParking.equals("CCW") ? true : false);
            }else if(driverOptions.getStartingPositionModes().contains("BLUE")) {
                driverOptions.setDuckParkingDirection(duckParking.equals("CW") ? false : true);
            }

            if(isDuckBySideRoute.equals("YES")){
                driverOptions.setDuckDeliverToHubBySideRoute(true);
            }else{
                driverOptions.setDuckDeliverToHubBySideRoute(false);
            }

            if(prefs.getString("Deliver to hub using openCV", "").equals("YES")){
                driverOptions.setDeliverToHubUsingOpencv(true);
            }else{
                driverOptions.setDeliverToHubUsingOpencv(false);
            }

            if(driverOptions.getStartingPositionModes().contains("RED")) {
                isRedAlliance = true;
            }


            Logger.logFile("starting position: " + driverOptions.getStartingPositionModes());
            Logger.logFile("parking: " + driverOptions.getParking());
            Logger.logFile("parking_delay_storage: " + driverOptions.getDelayParkingByStorageFromEnding());
            Logger.logFile("parking_delay_warehouse: " + driverOptions.getDelayParkingByWarehouseFromEnding());
            Logger.logFile("park only: " + driverOptions.getParkingOnly());
            Logger.logFile("Is duck parking counter clockwise: " + driverOptions.isDuckParkingCCW());
            Logger.logFile("feight delivery count:"+ driverOptions.getFreightDeliveryCount());
            Logger.logFile("setDeliver to hub using opencv:" + driverOptions.isDeliverToHubUsingOpencv());
        }
        catch (Exception e) {
            RobotLog.e("SharedPref exception " + e);
            this.delay = 0;
        }
        Logger.logFile("Done with init in autonomous");

    }

    @Override
    public void runOpMode() {
        initRobot();
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        robotHardware.getRobotVision().initWebCam("Webcam", true);  //boolean isRed

        RobotVision.AutonomousGoal goal = RobotVision.AutonomousGoal.NONE;
        Logger.logFile("recognition result: " + goal);

        AutonomousTaskBuilder builder = new AutonomousTaskBuilder(driverOptions, robotHardware, robotProfile, goal);
        taskList = builder.buildTaskList(goal);

        TaskReporter.report(taskList);
        Logger.logFile("Task list items: " + taskList.size());
        Logger.flushToFile();

        long loopStart = System.currentTimeMillis();
        long loopCnt = 0;
        try {
            Thread.sleep(1000);
        }
        catch (Exception ex) {};
        robotHardware.resetImu();
        robotHardware.robotVision.startWebcam("Webcam", null);
        startPos = robotProfile.getProfilePose(driverOptions.getStartingPositionModes() + "_START");
        robotHardware.getLocalizer().setPoseEstimate(startPos);
//        robotHardware.getMecanumDrive().setPoseEstimate(getProfilePose("START_STATE"));

        if (taskList.size() > 0) {
            taskList.get(0).prepare();
        }
        robotHardware.setMotorStopBrake(true);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && taskList.size()>0) {
            loopCount++;

            robotHardware.clearBulkCache();
            robotHardware.getLocalizer().update();

            if (taskList.size() > 0) {
                taskList.get(0).execute();

                if (taskList.get(0).isDone()) {
                    Logger.logFile("MainTaskComplete: " + taskList.get(0) + " Pose:" + robotHardware.getLocalizer().getPoseEstimate());
                    Logger.flushToFile();

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
        // Regardless, open the clamp to save the servo
        try {
            Logger.logFile("Autonomous - Final Location:" + robotHardware.getLocalizer().getPoseEstimate());
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }

        robotHardware.getRobotVision().stopWebcam("Webcam");
        robotHardware.stopAll();
        robotHardware.setMotorStopBrake(false);
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

}
