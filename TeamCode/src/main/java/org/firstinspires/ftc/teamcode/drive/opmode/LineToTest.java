package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotFactory;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class LineToTest extends LinearOpMode {
    public static double DISTANCE = 62; // in

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
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        robotHardware.resetDriveAndEncoders();
        //robotHardware.calibrateNavxGyro(telemetry);
        robotHardware.resetImu();

        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(DISTANCE/Math.sqrt(2), DISTANCE/Math.sqrt(2)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajectory);
        Logger.logFile("Final:" + robotHardware.getLocalizer().getPoseEstimate());
    }
}
