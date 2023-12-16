package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;


/**
 * 2019.10.26
 * Created by Ian Q.
 */
@TeleOp(name="AutonomousTest", group="Test")
public class AutonomousGenericTest extends LinearOpMode {

    RobotHardware robotHardware;
    RobotProfile robotProfile;

    Pose2d p0 = new Pose2d(12, 63, Math.PI/2);
    ArrayList<RobotControl> taskList;
    long loopCount = 0;
    int countTasks = 0;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private AprilTagRecognition aprilTagRecognition;
    private RobotCVProcessor.TEAM_PROP_POS team_prop_pos = RobotCVProcessor.TEAM_PROP_POS.RIGHT;
    String startPosMode;
    boolean isRed;

    public void initRobot() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profileA.json"));
        } catch (Exception e) {
        }
        Logger.init();
        Logger.init();

        RobotFactory.reset();
        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        Logger.logFile("Init completed");
    }

    @Override
    public void runOpMode() {
        initRobot();
        try {
            robotHardware.setMotorStopBrake(false); // so we can adjust the robot
            robotHardware.enableManualCaching(false);
//            robotHardware.initSetup(this);
//            robotHardware.initSetupNoAuto(this);
            robotHardware.resetLiftPos();
            Logger.logFile("init lift, set position to 0 =>" + robotHardware.getLiftPosition() );
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
        startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
        isRed = startPosMode.startsWith("RED");
        long loopStart = System.currentTimeMillis();
        aprilTagRecognition = new AprilTagRecognition(true, hardwareMap);
        aprilTagRecognition.initAprilTag();
        Logger.logFile("Webcam 1 status = "+ aprilTagRecognition.visionPortal.getCameraState());

        //RobotCVProcessor cvp = new RobotCVProcessor(robotHardware, robotProfile, true);
        //cvp.initWebCam("Webcam 1", true);

        int loopCnt = 0;
        while (!isStopRequested() && !isStarted()) {
            //RobotVision.AutonomousGoal goal = robotHardware.getRobotVision().getAutonomousRecognition(false);
            //telemetry.addData("goal",goal);
            //robotHardware.getLocalizer().update();
            //Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt%1000==0) {
                //telemetry.addData("CurrPose", currPose);
                telemetry.addData("LoopTPS", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                //telemetry.addData("Frame rate:", cvp.getFrameRate());
                telemetry.update();
                //cvp.saveNextImage();
            }
        }
        //cvp.stopStreaming();
        //cvp.close();
        //aprilTagSignalRecognition.stopRecognition();
        robotHardware.resetDriveAndEncoders();
        p0 = robotProfile.getProfilePose( isRed ? "START_POSE_RED_LEFT" : "START_POSE_BLUE_LEFT");
        p0 = new Pose2d(0,0,0);
        robotHardware.getLocalizer().setPoseEstimate(p0);
        taskList = new ArrayList<RobotControl>();

        //setupTaskList1();
        setupTaskList3();

        robotHardware.setMotorStopBrake(true);
        TaskReporter.report(taskList);
        Logger.logFile("Task list items: " + taskList.size());
        Logger.flushToFile();
        TaskReporter.report(taskList);


        if (taskList.size()>0) {
            Logger.logFile("Task Prepare " + taskList.get(0));
            taskList.get(0).prepare();
        }
        robotHardware.setMotorStopBrake(true);
        robotHardware.enableManualCaching(true);

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
            /*Specific test for motor velocity */
// append to shooting velocity csv file
            /* End Testing code */
            if (taskList.size() > 0) {
                taskList.get(0).execute();
                if (taskList.get(0).isDone()) {
                    Logger.logFile("MainTaskComplete: " + taskList.get(0) + " Pose:" + robotHardware.getLocalizer().getPoseEstimate()
                            + " gyro:" + robotHardware.getGyroHeading());
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
        try {
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }
    }

    void setupTaskList1() {
//        DriveConstraints constraints = new DriveConstraints(5.0, 5.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
//        DriveConstraints moveFast = new DriveConstraints(30.0, 20.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        velConstraint = getVelocityConstraint(15, 10, TRACK_WIDTH);
        accelConstraint = getAccelerationConstraint(10);
        // move to shoot
        Pose2d p1 = new Pose2d(9, 39, Math.PI/4 + Math.PI);
        Pose2d p1b= new Pose2d(12, 39,Math.PI/4);
        Pose2d p2 = new Pose2d(49, 39, 0);

        Trajectory trj = robotHardware.mecanumDrive.trajectoryBuilder(p0, true)
                .splineTo(p1.vec(), p1.getHeading(), velConstraint, accelConstraint)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trj);
        taskList.add(new RobotSleep(1000));
        taskList.add(moveTask1);
        taskList.add(new DropSpikeMarkTask(robotHardware));
        Trajectory trj2 = robotHardware.mecanumDrive.trajectoryBuilder(p1b)
                .splineTo(p2.vec(), p2.getHeading(), velConstraint, accelConstraint)
                .build();
        SplineMoveTask moveTask2= new SplineMoveTask(robotHardware.mecanumDrive, trj2);
        taskList.add(moveTask2);
        taskList.add(new RobotSleep(1000));
        taskList.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.CLOSE));
        if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT))
            taskList.add(new PixelUpTask(robotHardware, true, robotProfile.hardwareSpec.liftOutMin));
        else
            taskList.add(new PixelUpTask(robotHardware, false, robotProfile.hardwareSpec.liftOutMin));
        taskList.add(new RobotSleep(1000));
        taskList.add(new DropPixelTask(robotHardware));
    }

    void setupTaskList2() {
//        taskList.add(new RobotSleep((1000)));
//        taskList.add(new DropSpikeMarkTask(robotHardware));
//        taskList.add(new EqualDistanceTask(robotHardware,robotHardware.mecanumDrive, robotProfile));
        Pose2d parkingPose = robotProfile.getProfilePose("PARKING_" + startPosMode);
        Logger.logFile("parking pose:"+parkingPose.getX() +", "+parkingPose.getY()+", "+parkingPose.getHeading());

        taskList.add(new AprilTagDetectionTask(robotHardware, aprilTagRecognition,
                    robotProfile, team_prop_pos, robotHardware.mecanumDrive, isRed));
        taskList.add(new RobotSleep(1000));
        taskList.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.CLOSE));
        if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT)){
            taskList.add(new PixelUpTask(robotHardware, true, robotProfile.hardwareSpec.liftOutMin));
        }else{
            taskList.add(new PixelUpTask(robotHardware, false, robotProfile.hardwareSpec.liftOutMin));
        }

        taskList.add(new RobotSleep(1000));
        taskList.add(new DropPixelTask(robotHardware));
//        TrajectorySequence trajParking = robotHardware.mecanumDrive.trajectorySequenceBuilder(
//                                        robotHardware.mecanumDrive.getPoseEstimate())
//                                        .lineTo(parkingPose.vec())
//                                        .build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, parkingPose, false));
    }

    void setupTaskList3() {
        taskList.add(new IntakePositionTask(robotHardware, true));
        taskList.add(new IntakeActionTask(robotHardware, RobotHardware.IntakeMode.SLOW));
        TrajectorySequenceBuilder tb1 = robotHardware.mecanumDrive.trajectorySequenceBuilder(p0);
        tb1.back(6);
        TrajectorySequence ts1 = tb1.build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, ts1));
        taskList.add(new RobotSleep(500, "Take from top"));
        TrajectorySequenceBuilder tb2 = robotHardware.mecanumDrive.trajectorySequenceBuilder(ts1.end());
        tb2.setAccelConstraint(getAccelerationConstraint(5));
        tb2.setVelConstraint(getVelocityConstraint(5, Math.toRadians(5), 14.0));
        tb2.forward(2);
        TrajectorySequence ts2 = tb2.build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, ts2));
        taskList.add(new IntakePositionTask(robotHardware, false));
        taskList.add(new RobotSleep(500, "Intake"));
        TrajectorySequenceBuilder tb3 = robotHardware.mecanumDrive.trajectorySequenceBuilder(ts2.end());
        tb2.setAccelConstraint(getAccelerationConstraint(5));
        tb2.setVelConstraint(getVelocityConstraint(5, Math.toRadians(5), 14.0));
        tb3.forward(4);
        TrajectorySequence ts3 = tb3.build();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, ts3));
        taskList.add(new IntakeActionTask(robotHardware, RobotHardware.IntakeMode.ON));
        taskList.add(new RobotSleep(3000, "Into next tray"));
        taskList.add(new IntakeActionTask(robotHardware, RobotHardware.IntakeMode.OFF));
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

}
