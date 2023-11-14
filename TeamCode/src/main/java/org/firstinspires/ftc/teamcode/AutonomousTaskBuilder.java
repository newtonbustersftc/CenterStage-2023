package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.text.InputFilter;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Vector;

//almost worked version for high pole first and then low pole and possible ground junction
public class AutonomousTaskBuilder {
    RobotProfile robotProfile;
    RobotHardware robotHardware;
    ArrayList<RobotControl> taskList = new ArrayList<>();
    NBMecanumDrive drive;
    String delayString;
    String startPosMode;
    RobotCVProcessor.TEAM_PROP_POS team_prop_pos;  //= RobotCVProcessor.TEAM_PROP_POS.CENTER;

    public AutonomousTaskBuilder(RobotHardware robotHardware, RobotProfile robotProfile, RobotCVProcessor.TEAM_PROP_POS teamPropPos) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
        this.team_prop_pos = teamPropPos;
    }

    public ArrayList<RobotControl> buildTaskList() {
        try {
            SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
            delayString = prefs.getString(AutonomousOptions.START_DELAY_PREF, "0").replace(" sec", "");
            startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
            Logger.logFile(AutonomousOptions.START_POS_MODES_PREF + " - " + startPosMode);
            Logger.logFile(AutonomousOptions.START_DELAY_PREF + " - " + delayString);
//            Logger.logFile(AutonomousOptions.PARKING_PREF + " - " + parkingRow);
        } catch (Exception e) {
            RobotLog.e("SharedPref exception " + e);
            this.delayString = "0";
        }
        Logger.logFile("Done with init in autonomous");
        boolean isRight = startPosMode.endsWith("RIGHT");
        RobotProfile.AutonParam param = robotProfile.autonParam;
        TrajectoryVelocityConstraint velFast = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), robotProfile.hardwareSpec.trackWidth);
        TrajectoryAccelerationConstraint accelFast = getAccelerationConstraint(param.fastAcceleration);
        TrajectoryVelocityConstraint velConstraint = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), 3);
        TrajectoryAccelerationConstraint accelConstraint = getAccelerationConstraint(param.normAcceleration);

        if (!delayString.equals("0")) {
            taskList.add(new RobotSleep(Integer.parseInt(delayString) * 1000));
        }

        Pose2d prop_left = robotProfile.getProfilePose("TEAM_PROP_POS_LEFT");
        Pose2d prop_center = robotProfile.getProfilePose("TEAM_PROP_POS_CENTER");
        Pose2d prop_right = robotProfile.getProfilePose("TEAM_PROP_POS_RIGHT");
        Pose2d droppBoard_aprilTag_left = robotProfile.getProfilePose("DROPBOARD_APRILTAG_LEFT");
        Pose2d droppBoard_aprilTag_center = robotProfile.getProfilePose("DROPBOARD_APRILTAG_CENTER");
        Pose2d droppBoard_aprilTag_right = robotProfile.getProfilePose("DROPBOARD_APRILTAG_RIGHT");


        TrajectorySequence team_prop_pos_traj,dropBoard_traj_a, dropBoard_traj_b;

        if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT)){
            team_prop_pos_traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .setReversed(true)
                    .splineTo(prop_left.vec(), prop_left.getHeading()+Math.PI)
                    .build();
            dropBoard_traj_a = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                    .setReversed(true)
                    .lineTo(new Vector2d(-8,-12))
                    .strafeRight(13)
                    .build();
            dropBoard_traj_b = drive.trajectorySequenceBuilder(dropBoard_traj_a.end())
                    .setReversed(true)
                    .splineTo(droppBoard_aprilTag_left.vec(), droppBoard_aprilTag_left.getHeading()+Math.PI)
                    .build();
        }else if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.CENTER)){
            team_prop_pos_traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .setReversed(true)
                    .splineTo(prop_center.vec(), prop_center.getHeading()+Math.PI)
                    .build();
            dropBoard_traj_a = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                    .lineTo(new Vector2d(-15,-5))
                    .build();
            dropBoard_traj_b = drive.trajectorySequenceBuilder(dropBoard_traj_a.end())
                    .setReversed(true)
                    .splineTo(droppBoard_aprilTag_center.vec(), droppBoard_aprilTag_center.getHeading()+Math.PI)
                    .build();
        }else if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT)){
            team_prop_pos_traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .setReversed(true)
                    .strafeRight(8)
                    .splineTo(prop_right.vec(), prop_right.getHeading()+Math.PI)
                    .build();
            dropBoard_traj_a = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                    .strafeLeft(8)
                    .build();
            dropBoard_traj_b = drive.trajectorySequenceBuilder(dropBoard_traj_a.end())
                    .setReversed(true)
                    .splineTo(droppBoard_aprilTag_right.vec(), droppBoard_aprilTag_right.getHeading()+Math.PI)
                    .build();
        }else{
            team_prop_pos_traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .setReversed(true)
                    .splineTo(new Vector2d(-28, -25),Math.toRadians(90)+Math.PI)
                    .build();
            dropBoard_traj_a=drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                    .back(1)
                    .build();
            dropBoard_traj_b=drive.trajectorySequenceBuilder(dropBoard_traj_a.end())
                    .back(1)
                    .build();
        }

        taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
        taskList.add(new SplineMoveTask(drive, dropBoard_traj_a));
        taskList.add(new SplineMoveTask(drive, dropBoard_traj_b));





        return taskList;
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
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
