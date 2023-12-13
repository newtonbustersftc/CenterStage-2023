package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotFactory;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 50; // in

    @Override
    public void runOpMode() throws InterruptedException {
        NBMecanumDrive drive;
        RobotProfile robotProfile = null;
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profileA.json"));
        } catch (Exception e) {
        }

        Logger.init();

        RobotHardware robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
//        RobotHardware robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotHardware.resetDriveAndEncoders();
        //robotHardware.calibrateNavxGyro(telemetry);
        robotHardware.resetImu();
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
        drive.setPoseEstimate(new Pose2d(0,0,0));
        Logger.logFile("Begin:" + robotHardware.getLocalizer().getPoseEstimate());
        Logger.logFile("Begin IMU:" + robotHardware.getGyroHeading());
//        Logger.logFile("FR Encoder:" + robotHardware.getFRMotorEncoderCnt());
//        Logger.logFile("FL Encoder:" + robotHardware.getFLMotorEncoderCnt());
//        Logger.logFile("RR Encoder:" + robotHardware.getRRMotorEncoderCnt());
//        Logger.logFile("RL Encoder:" + robotHardware.getRlMotorEncoderCnt());

        int loopCnt = 0;
        long loopStart = System.currentTimeMillis();
        Logger.logFile("1");
        while (!isStarted()) {
            robotHardware.clearBulkCache();
            robotHardware.getLocalizer().update();
            Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
            loopCnt++;
            if (loopCnt==10000) {
                // Set to 0,0,0 for once
                robotHardware.getLocalizer().setPoseEstimate(new Pose2d());
            }
            if (loopCnt%10000==0) {
                telemetry.addData("CurrPose", currPose);
                telemetry.addData("LoopTPS:", (loopCnt * 1000 / (System.currentTimeMillis() - loopStart)));
                telemetry.update();
            }
        }
        Logger.logFile("2");
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        TrajectoryVelocityConstraint velConstraint = getVelocityConstraint(10, 10, TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelConstraint = getAccelerationConstraint(10);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(DISTANCE, velConstraint, accelConstraint)
                .build();

        if (isStopRequested()) return;
        Logger.logFile("3");
        drive.followTrajectorySequence(trajectory);
        Logger.logFile("Final:" + robotHardware.getLocalizer().getPoseEstimate());
        Logger.logFile("Final IMU:" + robotHardware.getGyroHeading());

//        Logger.logFile("FR Encoder:" + robotHardware.getFRMotorEncoderCnt());
////        Logger.logFile("FL Encoder:" + robotHardware.getFLMotorEncoderCnt());
//        Logger.logFile("RR Encoder:" + robotHardware.getRRMotorEncoderCnt());
//        Logger.logFile("RL Encoder:" + robotHardware.getRlMotorEncoderCnt());
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
