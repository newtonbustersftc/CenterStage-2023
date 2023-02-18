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
    RobotVision robotVision;

    ArrayList<RobotControl> taskList;
    long loopCount = 0;
    int countTasks = 0;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;

    public void initRobot() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
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
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        robotHardware.enableManualCaching(false);
        //robotHardware.initSetup(this);
        robotHardware.initSetupNoAuto(this);
        robotHardware.setMotorStopBrake(false); // so we can adjust the robot
        robotVision = robotHardware.getRobotVision();
        long loopStart = System.currentTimeMillis();
        AprilTagSignalRecognition aprilTagSignalRecognition = new AprilTagSignalRecognition(robotVision);
        aprilTagSignalRecognition.startRecognition();
        int loopCnt = 0;
        while (!isStopRequested() && !isStarted()) {
            //RobotVision.AutonomousGoal goal = robotHardware.getRobotVision().getAutonomousRecognition(false);
            //telemetry.addData("goal",goal);
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt%1==0) {
                telemetry.addData("CurrPose", currPose);
                telemetry.addData("LoopTPS", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.addData("Recognition Result", aprilTagSignalRecognition.getRecognitionResult());
                telemetry.update();
            }
        }
        aprilTagSignalRecognition.stopRecognition();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        taskList = new ArrayList<RobotControl>();

        setupTaskList3();

        robotHardware.setMotorStopBrake(true);
        TaskReporter.report(taskList);
        Logger.logFile("Task list items: " + taskList.size());
        Logger.flushToFile();
        DcMotorEx turrMotor = hardwareMap.get(DcMotorEx.class,"Turret Motor");
        turrMotor.setPower(0);
        turrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (taskList.size()>0) {
            Logger.logFile("Task Prepare " + taskList.get(0));
            taskList.get(0).prepare();
        }

        Logger.logFile("Main Task Loop started");

        while (opModeIsActive()) {
            loopCount++;
            robotHardware.clearBulkCache();
            robotHardware.getLocalizer().update();
            //Logger.logFile("Pose:" + robotHardware.getLocalizer().getPoseEstimate());
            //Logger.logFile("Velocity:" + robotHardware.getLocalizer().getPoseVelocity());
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
        Pose2d p0 = new Pose2d(0,0,0);
        Pose2d p1 = new Pose2d(20, 0, 0);
        Pose2d p2 = new Pose2d(40, -10, -Math.PI/4);

        Trajectory trj = robotHardware.mecanumDrive.trajectoryBuilder(p0)
                .lineTo(p1.vec(), velConstraint, accelConstraint)
                //.splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask1 = new SplineMoveTask(robotHardware.mecanumDrive, trj);
        taskList.add(new RobotSleep(1000));
        taskList.add(moveTask1);
        taskList.add(new RobotSleep(1000));
        Trajectory trj2 = robotHardware.mecanumDrive.trajectoryBuilder(p1)
                .splineTo(p2.vec(), p2.getHeading(), velConstraint, accelConstraint)
                //.splineToSplineHeading(p2, p2.getHeading(), constraints)
                .build();
        SplineMoveTask moveTask2= new SplineMoveTask(robotHardware.mecanumDrive, trj2);
        taskList.add(moveTask2);
        taskList.add(new RobotSleep(1000));
    }

    void setupTaskList2() {
        taskList.add(new GrabberTask(robotHardware,false));
        taskList.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionFullOutPos));
        taskList.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftDropPos[5]));
        taskList.add(new RobotSleep(500));
        taskList.add(new TurnTurretMotionTask(robotHardware, -1750, 1.0));
        taskList.add(new RobotSleep(2000));
        taskList.add(new TurnTurretMotionTask(robotHardware, 1750, 1.0));
        taskList.add(new RobotSleep(2000));
        taskList.add(new TurnTurretMotionTask(robotHardware, 0, 1.0));
        taskList.add(new RobotSleep(2000));
        taskList.add(new TurnTurretTask(robotHardware, -1750));
        taskList.add(new RobotSleep(2000));
        taskList.add(new TurnTurretTask(robotHardware, 1750));
        taskList.add(new RobotSleep(2000));
        taskList.add(new TurnTurretTask(robotHardware, 0));
        taskList.add(new RobotSleep(2000));
        taskList.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftPickPos[0]));
    }

    void setupTaskList3() {
        taskList.add(new GrabberTask(robotHardware,false));
        taskList.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionFullOutPos));
        taskList.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftDropPos[5]));
        taskList.add(new RobotSleep(500));
        TurnTurretMotionTask t = new TurnTurretMotionTask(robotHardware, 2200, 1.0);
        t.setWriteCsv(true);
        taskList.add(t);
        taskList.add(new RobotSleep(2000));
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
