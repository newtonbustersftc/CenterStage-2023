package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        NBMecanumDrive drive;
        RobotProfile robotProfile = null;
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        Logger.init();
        RobotHardware robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();

        waitForStart();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajectory);
        Logger.logFile("Final:" + robotHardware.getLocalizer().getPoseEstimate());
    }
}
