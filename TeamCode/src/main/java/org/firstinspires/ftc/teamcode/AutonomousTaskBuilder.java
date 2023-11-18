package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

//almost worked version for high pole first and then low pole and possible ground junction
public class AutonomousTaskBuilder {
    RobotProfile robotProfile;    RobotHardware robotHardware;
    ArrayList<RobotControl> taskList = new ArrayList<>();
    NBMecanumDrive drive;
    String delayString;
    String startPosMode;
    String parking;
    Pose2d startingPose;
    Pose2d lastLocation;
    RobotCVProcessor.TEAM_PROP_POS team_prop_pos;  //= RobotCVProcessor.TEAM_PROP_POS.CENTER;
    TrajectorySequence dropBoard_traj_a=null, dropBoard_traj_b=null, dropBoard_traj_c=null;
    AprilTagRecognition aprilTagRecognition;
    boolean isFarSideLeft =false;
    TrajectorySequence  team_prop_pos_traj=null,dropBoard_traj=null,lastTrajectory=null;
    boolean isRed;

    public AutonomousTaskBuilder(RobotHardware robotHardware, RobotProfile robotProfile,
                                 RobotCVProcessor.TEAM_PROP_POS teamPropPos, Pose2d startingPose, AprilTagRecognition aprilTagRecognition) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
        this.team_prop_pos = teamPropPos;
        this.startingPose = startingPose;
        this.aprilTagRecognition = aprilTagRecognition;
    }

    public ArrayList<RobotControl> buildTaskList() {
        try {
            SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
            delayString = prefs.getString(AutonomousOptions.START_DELAY_PREF, "0").replace(" sec", "");
            startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
            parking = prefs.getString(AutonomousOptions.PARKING_PREF,AutonomousOptions.PARKING_LOCATION[0]);
            Logger.logFile(AutonomousOptions.START_POS_MODES_PREF + " - " + startPosMode);
            Logger.logFile(AutonomousOptions.START_DELAY_PREF + " - " + delayString);
            Logger.logFile(AutonomousOptions.PARKING_PREF + " - " + parking);
            if(startPosMode.startsWith("RED")){
                isRed = true;
            }
        } catch (Exception e) {
            RobotLog.e("SharedPref exception " + e);
            this.delayString = "0";
        }
        Logger.logFile("Done with init in autonomous");

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

        if(lastTrajectory == null){
            if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT)){
                if(startPosMode.startsWith("BLUE")){
                    lastLocation = new Pose2d(droppBoard_aprilTag_blue_left.vec(), droppBoard_aprilTag_blue_left.getHeading());
                }else {
                    lastLocation = new Pose2d(droppBoard_aprilTag_red_left.vec(), droppBoard_aprilTag_red_left.getHeading());
                }
            }else if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.CENTER)){
                if(startPosMode.startsWith("BLUE")){
                    lastLocation = new Pose2d(droppBoard_aprilTag_blue_center.vec(), droppBoard_aprilTag_blue_center.getHeading());
                }else {
                    lastLocation = new Pose2d(droppBoard_aprilTag_red_center.vec(), droppBoard_aprilTag_red_center.getHeading());
                }
            }else{
                if(startPosMode.startsWith("BLUE")){
                    lastLocation = new Pose2d(droppBoard_aprilTag_blue_right.vec(), droppBoard_aprilTag_blue_right.getHeading());
                }else {
                    lastLocation = new Pose2d(droppBoard_aprilTag_red_right.vec(), droppBoard_aprilTag_red_right.getHeading());
                }
            }
        }
        //Blue side share same drop board destination
        if(startPosMode.startsWith("BLUE")){
            if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT)){
                if(startPosMode.contains("LEFT")){   //"BLUE_LEFT" - near side
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                                .setReversed(true)
                                .splineTo(prop_pos_blueleft_left.vec(), prop_pos_blueleft_left.getHeading() + Math.PI)
                                .build();

                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
//                    taskList.add(new SpikeMarkDroppingStickTask(robotHardware));

                    dropBoard_traj = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                            .splineTo(droppBoard_aprilTag_blue_left.vec(), droppBoard_aprilTag_blue_left.getHeading())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj));
                    goToDropBoard();
                }else{                               //"BLUE_RIGHT - far side
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_blueright_left.vec(), prop_pos_blueright_left.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));

                    //todo, need to find the right path
                    isFarSideLeft = true;
//                    Pose2d pose_a = new Pose2d(-36,7, 0);
//                    Pose2d pose_b = new Pose2d(13, 12, 0);
//                    Pose2d pose_a = new Pose2d(-32,60, 0); //transition point by the wall
//                    Pose2d pose_b = new Pose2d(15, 60, 0);
//                    Pose2d pose_c = new Pose2d(38, 39, 0);
                    Pose2d pose_a = new Pose2d(-40,8, 0); //transition point by the center
                    Pose2d pose_b = new Pose2d(0, 10, 0);
                    Pose2d pose_c = new Pose2d(40, 27, 0);

                    goToDropBoard(pose_a,pose_b, pose_c, team_prop_pos_traj, team_prop_pos, isFarSideLeft);
                }
            }else if(team_prop_pos.equals( RobotCVProcessor.TEAM_PROP_POS.CENTER)){
                if(startPosMode.contains("LEFT")){ // "BLUE_LEFT"
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_blueleft_center.vec(), prop_pos_blueleft_center.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
                    dropBoard_traj = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                            .splineTo(droppBoard_aprilTag_blue_center.vec(), droppBoard_aprilTag_blue_center.getHeading())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj));
                    goToDropBoard();
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
//                    Pose2d pose_a = new Pose2d(-32,60, 0); //transition point
//                    Pose2d pose_b = new Pose2d(15, 60, 0);
                    Pose2d pose_a = new Pose2d(-50,38, 0); //transition point
                    Pose2d pose_b = new Pose2d(15, 38, 0);
//                    Pose2d pose_c = new Pose2d(36, 36, 0);
                    Pose2d pose_c = droppBoard_aprilTag_blue_center;

                    goToDropBoard(pose_a,pose_b, pose_c, team_prop_pos_traj, team_prop_pos, isFarSideLeft);
                }
            }else{  //must be team_prop_pos == RobotCVProcessor.TEAM_PROP_POS.RIGHT
                if(startPosMode.contains("LEFT")){ // "BLUE_LEFT"
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_blueleft_right.vec(), prop_pos_blueleft_right.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
                    dropBoard_traj = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                            .splineTo(droppBoard_aprilTag_blue_right.vec(), droppBoard_aprilTag_blue_right.getHeading())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj));
                    goToDropBoard();
                }else{                              // must be "BLUE_RIGHT"
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_blueright_right.vec(), prop_pos_blueright_right.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
//                    Pose2d pose_a = new Pose2d(-32,60, 0); //transition point
//                    Pose2d pose_b = new Pose2d(15, 60, 0);
//                    Pose2d pose_a = new Pose2d(-50,12, 45); //transition point
//                    Pose2d pose_b = new Pose2d(15, 12, 0);
                    Pose2d pose_a = new Pose2d(-32,38, 0); //transition point
                    Pose2d pose_b = new Pose2d(15, 38, 0);
//                    Pose2d pose_c = new Pose2d(36, 32, 0);
                    Pose2d pose_c = droppBoard_aprilTag_blue_right;
                    goToDropBoard(pose_a, pose_b, pose_c, team_prop_pos_traj, team_prop_pos, isFarSideLeft);
                }
            }
        }else{  //"RED" side
            if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT)){
                if(startPosMode.contains("LEFT")){   //"RED_LEFT" - far side
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                                .setReversed(true)
                                .splineTo(prop_pos_redleft_left.vec(), prop_pos_redleft_left.getHeading() + Math.PI)
                                .build();

                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
//                    taskList.add(new SpikeMarkDroppingStickTask(robotHardware));

//                    Pose2d pose_a = new Pose2d(-36,7, 0);
//                    Pose2d pose_b = new Pose2d(13, 12, 0);
                    Pose2d pose_a = new Pose2d(-32,-38, 0); //transition point
                    Pose2d pose_b = new Pose2d(15, -38, 0);
//                    Pose2d pose_c = new Pose2d(36, -32, 0);
                    Pose2d pose_c = droppBoard_aprilTag_red_left;

                    goToDropBoard(pose_a,pose_b, pose_c, team_prop_pos_traj, team_prop_pos, isFarSideLeft);

                }else{                               //"RED_RIGHT near side
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_redright_left.vec(), prop_pos_redright_left.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));

                    //go directly to dropboard with AprilTag detection
                    dropBoard_traj = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                            .splineTo(droppBoard_aprilTag_red_left.vec(), droppBoard_aprilTag_red_left.getHeading())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj));
                    goToDropBoard();
                }
            }else if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.CENTER)){
                if(startPosMode.contains("LEFT")){ // "RED_LEFT" - far side
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_redleft_center.vec(), prop_pos_redleft_center.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));

//                    Pose2d pose_a = new Pose2d(-50,12, 0); //through center
//                    Pose2d pose_b = new Pose2d(15, 12, 0);
//                    Pose2d pose_a = new Pose2d(-32,60, 0); //transition point
//                    Pose2d pose_b = new Pose2d(15, 60, 0);
                    Pose2d pose_a = new Pose2d(-32,-38, 0); //transition point
                    Pose2d pose_b = new Pose2d(15, -38, 0);
//                    Pose2d pose_c = new Pose2d(36, -40, 0);
                    Pose2d pose_c = droppBoard_aprilTag_red_center;

                    goToDropBoard(pose_a,pose_b, pose_c, team_prop_pos_traj, team_prop_pos, isFarSideLeft);
                }else{                              // must be "RED_RIGHT" near side
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_redright_center.vec(), prop_pos_redright_center.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
                    dropBoard_traj = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                            .splineTo(droppBoard_aprilTag_red_center.vec(), droppBoard_aprilTag_red_center.getHeading())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj));
                    goToDropBoard();
                }
            }else{   //must be team_prop_pos = RobotCVProcessor.TEAM_PROP_POS.RIGHT
                if(startPosMode.contains("LEFT")){ // "RED_LEFT" - far side
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_redleft_right.vec(), prop_pos_redleft_right.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));

                    //todo find the right path
//                    Pose2d pose_a = new Pose2d(-32,60, 0); //transition point
//                    Pose2d pose_b = new Pose2d(15, 60, 0);
//                    Pose2d pose_a = new Pose2d(-50,12, 45); //transition point
//                    Pose2d pose_b = new Pose2d(15, 12, 0);
//                    Pose2d pose_a = new Pose2d(-32,-38, 0); //transition point
//                    Pose2d pose_b = new Pose2d(15, -38, 0);
//                    Pose2d pose_c = new Pose2d(36, 32, 0);
                    Pose2d pose_a = new Pose2d(-40,-8, 0); //transition point by the center
                    Pose2d pose_b = new Pose2d(0, -8, 0);
                    Pose2d pose_c = new Pose2d(40, -30, 0);

                    isFarSideLeft =true;
                    goToDropBoard(pose_a, pose_b, pose_c, team_prop_pos_traj, team_prop_pos, isFarSideLeft);
                }else{                              // must be "RED_RIGHT" - near side
                    team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                            .setReversed(true)
                            .splineTo(prop_pos_redright_right.vec(), prop_pos_redright_right.getHeading() + Math.PI)
                            .build();
                    taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
                    taskList.add(new RobotSleep(1000));
                    taskList.add(new DropSpikeMarkTask(robotHardware));
                    dropBoard_traj = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                            .splineTo(droppBoard_aprilTag_red_right.vec(), droppBoard_aprilTag_red_right.getHeading())
                            .build();
                    taskList.add(new SplineMoveTask(drive, dropBoard_traj));
                    goToDropBoard();
                }
            }
        }
        lastTrajectory = dropBoard_traj; //for all

        //parking:
        Trajectory parkingTrajectory=null;

        int CORNER_LEFT=16, CORNER_CENTER=24, CORNER_RIGHT=32,
            WALL_LEFT = 26, WALL_CENTER=18, WALL_RIGHT=10;
        int forward=8;

        if(parking.equals("CORNER")){
            if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT)){
                parkingTrajectory = drive.trajectoryBuilder(lastTrajectory == null ? lastLocation : lastTrajectory.end())
                        .strafeLeft(CORNER_LEFT)
                        .forward(forward)
                        .build();
            }else if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.CENTER)){                              //must be "RED"
                parkingTrajectory = drive.trajectoryBuilder(lastLocation)
                        .strafeLeft(CORNER_CENTER)
                        .forward(forward)
                        .build();
            }else if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT)){
                parkingTrajectory = drive.trajectoryBuilder(lastLocation)
                        .strafeLeft(CORNER_RIGHT)
                        .forward(forward)
                        .build();
            }
        }else{                                  //must be "WALL"
            if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT)){  //"BLUE"
                parkingTrajectory = drive.trajectoryBuilder(lastLocation)
                        .strafeRight(WALL_LEFT)
                        .forward(forward)
                        .build();
            }else if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.CENTER)){                              //must be "RED"
                parkingTrajectory = drive.trajectoryBuilder(lastLocation)
                        .strafeRight(WALL_CENTER)
                        .forward(forward)
                        .build();
            }else if(team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT)){
                parkingTrajectory = drive.trajectoryBuilder(lastLocation)
                        .strafeRight(WALL_RIGHT)
                        .forward(forward)
                        .build();
            }
        }
        taskList.add(new SplineMoveTask(drive, parkingTrajectory));
        return taskList;
    }

    void goToDropBoard(Pose2d pose_a, Pose2d pose_b, Pose2d pose_c, TrajectorySequence team_prop_pos_traj,
                       RobotCVProcessor.TEAM_PROP_POS team_prop_pos, boolean isFarSideLeft){
        dropBoard_traj_a = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                .splineTo(pose_a.vec(), pose_a.getHeading())
                .build();
        taskList.add(new SplineMoveTask(drive, dropBoard_traj_a));
        dropBoard_traj_b = drive.trajectorySequenceBuilder(dropBoard_traj_a.end())
                .lineTo(pose_b.vec())
                .build();
        taskList.add(new SplineMoveTask(drive, dropBoard_traj_b));
        if(isFarSideLeft) {
            dropBoard_traj_c = drive.trajectorySequenceBuilder(dropBoard_traj_b.end())
                    .splineTo(pose_c.vec(),pose_c.getHeading())
                    .build();
            taskList.add(new AprilTagDetectionTask(robotHardware, this.aprilTagRecognition,  team_prop_pos, drive, isRed ));
            taskList.add(new SplineMoveTask(drive, robotHardware, isRed)); //which will call RobotHardware to get the desired TagID dynamically
        }else{
            dropBoard_traj_c = drive.trajectorySequenceBuilder(dropBoard_traj_b.end())
                    .lineTo(pose_c.vec())
                    .build();
            lastTrajectory = dropBoard_traj_c;
        }
        taskList.add(new SplineMoveTask(drive, dropBoard_traj_c));

        goToDropBoard();
    }

    void goToDropBoard(){
        taskList.add(new RobotSleep(1000));
        taskList.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.CLOSE));
        taskList.add(new PixelUpTask(robotHardware, robotProfile.hardwareSpec.liftOutMin));
        taskList.add(new RobotSleep(1000));
        taskList.add(new DropPixelTask(robotHardware));
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
