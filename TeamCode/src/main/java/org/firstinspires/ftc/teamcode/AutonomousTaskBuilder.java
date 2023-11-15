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
    Pose2d startingPose;
    RobotCVProcessor.TEAM_PROP_POS team_prop_pos;  //= RobotCVProcessor.TEAM_PROP_POS.CENTER;

    public AutonomousTaskBuilder(RobotHardware robotHardware, RobotProfile robotProfile,
                                 RobotCVProcessor.TEAM_PROP_POS teamPropPos, Pose2d startingPose) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
        this.team_prop_pos = teamPropPos;
        this.startingPose = startingPose;
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
//        boolean isRight = startPosMode.endsWith("RIGHT");
        RobotProfile.AutonParam param = robotProfile.autonParam;
        TrajectoryVelocityConstraint velFast = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), robotProfile.hardwareSpec.trackWidth);
        TrajectoryAccelerationConstraint accelFast = getAccelerationConstraint(param.fastAcceleration);
        TrajectoryVelocityConstraint velConstraint = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), 3);
        TrajectoryAccelerationConstraint accelConstraint = getAccelerationConstraint(param.normAcceleration);

        if (!delayString.equals("0")) {
            taskList.add(new RobotSleep(Integer.parseInt(delayString) * 1000));
        }

        Pose2d prop_pos_blueleft_left = robotProfile.getProfilePose("TEAM_PROP_POS_BLUELEFT_LEFT");
        Pose2d prop_pos_blueleft_center = robotProfile.getProfilePose("TEAM_PROP_POS_BLUELEFT_CENTER");
        Pose2d prop_pos_blueleft_right= robotProfile.getProfilePose("TEAM_PROP_POS_BLUELEFT_RIGHT");

        Pose2d prop_pos_blueright_left = robotProfile.getProfilePose("TEAM_PROP_POS_BLUERIGHT_LEFT");
        Pose2d prop_pos_blueright_center = robotProfile.getProfilePose("TEAM_PROP_POS_BLUERIGHT_CENTER");
        Pose2d prop_pos_blueright_right= robotProfile.getProfilePose("TEAM_PROP_POS_BLUERIGHT_RIGHT");

        Pose2d prop_pos_redleft_left = robotProfile.getProfilePose("TEAM_PROP_POS_REDLEFT_LEFT");
        Pose2d prop_pos_redleft_center= robotProfile.getProfilePose("TEAM_PROP_POS_REDLEFT_CENTER");
        Pose2d prop_pos_redleft_right= robotProfile.getProfilePose("TEAM_PROP_POS_REDLEFT_RIGHT");

        Pose2d prop_pos_redright_left= robotProfile.getProfilePose("TEAM_PROP_POS_REDRIGHT_LEFT");
        Pose2d prop_pos_redright_center = robotProfile.getProfilePose("TEAM_PROP_POS_REDRIGHT_CENTER");
        Pose2d prop_pos_redright_right= robotProfile.getProfilePose("TEAM_PROP_POS_REDRIGHT_RIGHT");

        Pose2d droppBoard_aprilTag_blue_left = robotProfile.getProfilePose("DROPBOARD_APRILTAG_BLUE_LEFT");
        Pose2d droppBoard_aprilTag_blue_center = robotProfile.getProfilePose("DROPBOARD_APRILTAG_BLUE_CENTER");
        Pose2d droppBoard_aprilTag_blue_right = robotProfile.getProfilePose("DROPBOARD_APRILTAG_BLUE_RIGHT");

        Pose2d droppBoard_aprilTag_red_left = robotProfile.getProfilePose("DROPBOARD_APRILTAG_RED_LEFT");
        Pose2d droppBoard_aprilTag_red_center = robotProfile.getProfilePose("DROPBOARD_APRILTAG_RED_CENTER");
        Pose2d droppBoard_aprilTag_red_right = robotProfile.getProfilePose("DROPBOARD_APRILTAG_RED_RIGHT");

        TrajectorySequence team_prop_pos_traj=null,dropBoard_traj=null, dropBoard_traj_a=null, dropBoard_traj_b=null;

        //Blue side share same drop board destination
        if(startPosMode.startsWith("BLUE")){
            if(team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.LEFT){
                if(startPosMode.contains("LEFT")){   //"BLUE_LEFT"
                    if(team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.LEFT) {
                        team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                                .setReversed(true)
                                .splineTo(prop_pos_blueleft_left.vec(), prop_pos_blueleft_left.getHeading() + Math.PI)
                                .build();
                    }else if(team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.CENTER){
                        team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                                .setReversed(true)
                                .splineTo(prop_pos_blueleft_center.vec(), prop_pos_blueleft_center.getHeading() + Math.PI)
                                .build();
                    }else {   //must be team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.RIGHT
                        team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                                .setReversed(true)
                                .splineTo(prop_pos_blueleft_right.vec(), prop_pos_blueleft_right.getHeading() + Math.PI)
                                .build();
                    }
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
                }else{                               //"BLUE_RIGHT
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_blueright_left.vec(), prop_pos_blueright_left.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));

                    //todo make a decision whether go under through middle/center or by the wall(alliance might block)
                    Pose2d pose_a = new Pose2d(-50,12, 0); //through center
                    Pose2d pose_b = new Pose2d(15, 12, 0);
                    dropBoard_traj_a = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                            .splineTo(pose_a.vec(), pose_a.getHeading())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj_a));
                    dropBoard_traj_b = drive.trajectorySequenceBuilder(dropBoard_traj_a.end())
                            .lineTo(pose_b.vec())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj_b));
                }
                //todo, the BLUE_RIGHT "left" always go to dropboard "center"
                //both BLUE sides all share same drop board "left" after drop team_prop on spike mark:
                dropBoard_traj = drive.trajectorySequenceBuilder(startPosMode.contains("LEFT")?team_prop_pos_traj.end():dropBoard_traj_b.end())
                        .splineTo(droppBoard_aprilTag_blue_left.vec(), droppBoard_aprilTag_blue_left.getHeading())
                        .build();
                taskList.add(new SplineMoveTask(drive, dropBoard_traj));
                taskList.add(new RobotSleep(1000));
                taskList.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.CLOSE));
                taskList.add(new PixelUpTask(robotHardware, robotProfile.hardwareSpec.liftOutMin));
                taskList.add(new RobotSleep(1000));
                taskList.add(new DropPixelTask(robotHardware));

            }else if(team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.CENTER){
                if(startPosMode.contains("LEFT")){ // "BLUE_LEFT"
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_blueleft_center.vec(), prop_pos_blueleft_center.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
                }else{                              // must be "BLUE_RIGHT"
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_blueright_center.vec(), prop_pos_blueright_center.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));

//                    Pose2d pose_a = new Pose2d(-50,12, 0); //through center
//                    Pose2d pose_b = new Pose2d(15, 12, 0);
                    Pose2d pose_a = new Pose2d(-32,60, 0); //transition point
                    Pose2d pose_b = new Pose2d(15, 60, 0);
                    dropBoard_traj_a = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                            .splineTo(pose_a.vec(), pose_a.getHeading())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj_a));
                    dropBoard_traj_b = drive.trajectorySequenceBuilder(dropBoard_traj_a.end())
                            .lineTo(pose_b.vec())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj_b));
                }

                //both BLUE sides all going to drop board "center" after drop team_prop on spike mark:
                dropBoard_traj = drive.trajectorySequenceBuilder(startPosMode.contains("LEFT")?team_prop_pos_traj.end():dropBoard_traj_b.end())
                        .splineTo(droppBoard_aprilTag_blue_center.vec(), droppBoard_aprilTag_blue_center.getHeading())
                        .build();
                taskList.add(new SplineMoveTask(drive, dropBoard_traj));
                taskList.add(new RobotSleep(1000));
                taskList.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.CLOSE));
                taskList.add(new PixelUpTask(robotHardware, robotProfile.hardwareSpec.liftOutMin));
                taskList.add(new RobotSleep(1000));
                taskList.add(new DropPixelTask(robotHardware));
            }else{  //must be team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.RIGHT
                if(startPosMode.contains("LEFT")){ // "BLUE_LEFT"
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_blueleft_right.vec(), prop_pos_blueleft_right.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
                }else{                              // must be "BLUE_RIGHT"
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_blueright_right.vec(), prop_pos_blueright_right.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
                    Pose2d pose_a = new Pose2d(-32,60, 0); //transition point
                    Pose2d pose_b = new Pose2d(15, 60, 0);

                    dropBoard_traj_a = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                            .splineTo(pose_a.vec(), pose_a.getHeading())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj_a));
                    dropBoard_traj_b = drive.trajectorySequenceBuilder(dropBoard_traj_a.end())
                            .lineTo(pose_b.vec())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj_b));
                }

                //both BLUE sides all going to drop board "right" after drop team_prop on spike mark:
                dropBoard_traj = drive.trajectorySequenceBuilder(startPosMode.contains("LEFT")?team_prop_pos_traj.end():dropBoard_traj_b.end())
                        .splineTo(droppBoard_aprilTag_blue_right.vec(), droppBoard_aprilTag_blue_right.getHeading())
                        .build();
                taskList.add(new SplineMoveTask(drive, dropBoard_traj));
                taskList.add(new RobotSleep(1000));
                taskList.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.CLOSE));
                taskList.add(new PixelUpTask(robotHardware, robotProfile.hardwareSpec.liftOutMin));
                taskList.add(new RobotSleep(1000));
                taskList.add(new DropPixelTask(robotHardware));
            }
        }else{  //"RED" side
            if(team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.LEFT){
                if(startPosMode.contains("LEFT")){  //"RED_LEFT"

                }else{                              //"RED_RIGHT"

                }
            }else if(team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.CENTER){
                if(startPosMode.contains("LEFT")){  //"RED_LEFT"

                }else{                              //"RED_RIGHT"

                }
            }else{   //must be team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.RIGHT
                if(startPosMode.contains("LEFT")){   //"RED_LEFT

                }else{                               //"RED_RIGHT"

                }
            }
        }

        //parking:



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
