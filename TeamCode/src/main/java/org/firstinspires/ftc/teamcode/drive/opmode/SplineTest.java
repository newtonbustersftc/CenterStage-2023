package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;
import java.util.Arrays;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;

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

        telemetry.clearAll();
        waitForStart();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));

        if (isStopRequested()) return;
        Logger.logFile("Pose2d: "+ drive.getPoseEstimate());
        Logger.flushToFile();
//        DriveConstraints constraints = new DriveConstraints(20.0, 15.0, 0.0, Math.toRadians(360.0), Math.toRadians(360.0), 0.0);
        velConstraint = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
        accelConstraint = getAccelerationConstraint(MAX_ACCEL);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0,0, 0))
                .splineTo(new Vector2d(20, 0), 0)
                .splineTo(new Vector2d(45, 25), 0)
                .splineTo(new Vector2d(70, -5), 0)
                .waitSeconds(2)
                .build();

        drive.followTrajectorySequence(traj);
        Logger.logFile("Pose2d:"+ drive.getPoseEstimate());

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(70,-5, 0))
                .setReversed(true)
                .splineTo(new Vector2d(30, -20), Math.PI)
                .splineTo(new Vector2d(0,0),Math.PI)
                .build();
        drive.followTrajectorySequence(traj2);
        sleep(2000);
        Logger.logFile("Pose2d:"+ drive.getPoseEstimate());
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
