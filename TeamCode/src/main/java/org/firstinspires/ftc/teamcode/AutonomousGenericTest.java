package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

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
            //robotHardware.initSetup(this);
            //robotHardware.initSetupNoAuto(this);
            robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }
        long loopStart = System.currentTimeMillis();
        //AprilTagSignalRecognition aprilTagSignalRecognition = new AprilTagSignalRecognition(robotVision);
        //aprilTagSignalRecognition.startRecognition();
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
        robotHardware.getLocalizer().setPoseEstimate(p0);
        taskList = new ArrayList<RobotControl>();

        setupTaskList1();

        robotHardware.setMotorStopBrake(true);
        TaskReporter.report(taskList);
        Logger.logFile("Task list items: " + taskList.size());
        Logger.flushToFile();


        if (taskList.size()>0) {
            Logger.logFile("Task Prepare " + taskList.get(0));
            taskList.get(0).prepare();
        }

        Logger.logFile("Main Task Loop started");

        while (opModeIsActive()) {
            loopCount++;
            robotHardware.clearBulkCache();
            robotHardware.getLocalizer().update();
            Logger.logFile("Pose:" + robotHardware.getLocalizer().getPoseEstimate());
            Logger.logFile("Velocity:" + robotHardware.getLocalizer().getPoseVelocity());
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
        taskList.add(new PixelUpTask(robotHardware, robotProfile.hardwareSpec.liftOutMin));
        taskList.add(new RobotSleep(1000));
        taskList.add(new DropPixelTask(robotHardware));
    }

    void setupTaskList2() {
    }

    void setupTaskList3() {
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
