package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotFactory;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class FollowerPIDTuner extends LinearOpMode {
    public static double DISTANCE = 48; // in

    @Override
    public void runOpMode() throws InterruptedException {
        NBMecanumDrive drive;
        RobotProfile robotProfile = null;
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        Logger.init();
        RobotHardware robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            robotHardware.clearBulkCache();
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .forward(DISTANCE)
                    .build();
            drive.followTrajectory(traj);
            drive.turn(Math.toRadians(90));

            startPose = traj.end().plus(new Pose2d(0, 0, Math.toRadians(90)));
        }
    }
}
