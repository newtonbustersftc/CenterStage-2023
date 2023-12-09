package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;
import java.util.Arrays;

//almost worked version for high pole first and then low pole and possible ground junction
public class AutonomousTaskBuilder {
    RobotProfile robotProfile;    RobotHardware robotHardware;
    ArrayList<RobotControl> taskList = new ArrayList<>();
    NBMecanumDrive drive;
    String delayString, startPosMode, passThrough, parking, wayPoint = "_WAY_POINT";
    Pose2d startingPose, wp1, wp2, wp3, wp4, wp5, wp6, wp7,aprilTagPt;
    RobotCVProcessor.TEAM_PROP_POS teamPropPos = RobotCVProcessor.TEAM_PROP_POS.CENTER; //default in case
    AprilTagRecognition aprilTagRecognition;
    TrajectorySequence  team_prop_pos_traj=null,dropBoard_traj=null;
    boolean isRed, isFar, isSkipWeightPoint, isStraightToSpikeMark;

    public AutonomousTaskBuilder(RobotHardware robotHardware, RobotProfile robotProfile,
                                 RobotCVProcessor.TEAM_PROP_POS teamPropPos, Pose2d startingPose,
                                 AprilTagRecognition aprilTagRecognition, String parking) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
        this.teamPropPos = teamPropPos;
        this.startingPose = startingPose;
        this.aprilTagRecognition = aprilTagRecognition;
        this.parking = parking;
    }

    public ArrayList<RobotControl> buildTaskList() {
        try {
            SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
            delayString = prefs.getString(AutonomousOptions.START_DELAY_PREF, "0").replace(" sec", "");
            startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
            passThrough = prefs.getString(AutonomousOptions.PASS_PREF, AutonomousOptions.PASS_THROUGH[0]);
            Logger.logFile(AutonomousOptions.START_POS_MODES_PREF + " - " + startPosMode);
            Logger.logFile(AutonomousOptions.START_DELAY_PREF + " - " + delayString);
            isRed = startPosMode.startsWith("RED") ? true : false;
            isFar = (startPosMode.equals("BLUE_RIGHT") || startPosMode.equals("RED_LEFT"));
            isSkipWeightPoint = teamPropPos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT) && startPosMode.equals("BLUE_LEFT") ||
                                teamPropPos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT) && startPosMode.equals("RED_RIGHT");
            isStraightToSpikeMark = teamPropPos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT) && startPosMode.equals("BLUE_RIGHT") ||
                                    teamPropPos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT) && startPosMode.equals("RED_LEFT") ||
                                    teamPropPos.equals(RobotCVProcessor.TEAM_PROP_POS.CENTER) && isFar;
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
        Pose2d propPose = robotProfile.getProfilePose("TEAM_PROP_POS_" + teamPropPos + "_" +startPosMode);
        Pose2d dropPose = null;
        if(!isFar ) {
            if (!isSkipWeightPoint) {
                dropPose = robotProfile.getProfilePose("WAY_POINT_" + (isRed ? "RED_APRILTAG_A" : "BLUE_APRILTAG_A"));
            } else {
                dropPose = robotProfile.getProfilePose("WAY_POINT_" + (isRed ? "RED_RIGHT_B" : "BLUE_LEFT_B"));
            }
        }
        Pose2d parkingPose1 = null, parkingPose2 = null;
        if(parking.equals("CORNER")){
            parkingPose1 = isRed ? robotProfile.getProfilePose("PARKING_RED_CORNER") :
                                    robotProfile.getProfilePose("PARKING_BLUE_CORNER");
        }else{
            parkingPose1 = isRed ? robotProfile.getProfilePose("PARKING_RED_MIDDLE") :
                    robotProfile.getProfilePose("PARKING_BLUE_MIDDLE");
        }
        parkingPose2 = new Pose2d(parkingPose1.getX()+5, parkingPose1.getY(), parkingPose1.getHeading());

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startingPose);
                    builder.setReversed(true);
                    if(isStraightToSpikeMark){
                        builder.lineTo(propPose.vec());
                    }else {
                        builder.splineTo(propPose.vec(), propPose.getHeading() + Math.PI);
                    }

        team_prop_pos_traj = builder.build();
        taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
        taskList.add(new RobotSleep(1000));
        taskList.add(new DropSpikeMarkTask(robotHardware));

        if (!isFar) {
            dropBoard_traj = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                    .splineTo(dropPose.vec(), dropPose.getHeading())
                    .build();
            taskList.add(new SplineMoveTask(drive, dropBoard_traj));


        }else { // far sides
            wp1 = robotProfile.getProfilePose(passThrough + wayPoint + "1_" + teamPropPos + "_" +startPosMode);
            wp2 = robotProfile.getProfilePose(passThrough + wayPoint + "2_" + teamPropPos + "_" +startPosMode);
            wp3 = robotProfile.getProfilePose(passThrough + wayPoint + "3_" + teamPropPos + "_" +startPosMode);
            wp4 = robotProfile.getProfilePose(passThrough + wayPoint + "4_" + teamPropPos + "_" +startPosMode);
            wp5 = robotProfile.getProfilePose(passThrough + wayPoint + "5_" + teamPropPos + "_" +startPosMode);
            wp6 = robotProfile.getProfilePose(passThrough + wayPoint + "6_" + teamPropPos + "_" +startPosMode);
            wp7 = robotProfile.getProfilePose(passThrough + wayPoint + "7_" + teamPropPos + "_" +startPosMode);

            String color = isRed ? "RED":"BLUE";
            if(!passThrough.equals("WALL")) {
                aprilTagPt = isRed ? new Pose2d(38, -42, 0) : new Pose2d(38, 38, 0);
            }else {
                aprilTagPt = isRed ? robotProfile.getProfilePose("WAY_POINT_" + color + "_APRILTAG_A") :
                            new Pose2d(38,35, 0);
            }
            if (teamPropPos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT)) {
                    if(isRed) {
                        buildRightTrajectory();
                    }else{
                        buildLeftTrajectory();
                    }
            } else if (teamPropPos.equals(RobotCVProcessor.TEAM_PROP_POS.CENTER)) {
                    buildCenterTrajectory();
            } else {
                    if(isRed){
                        buildLeftTrajectory();
                    }else{
                        buildRightTrajectory();
                    }
            }
        }
        taskList.add(new AprilTagDetectionTask(robotHardware, this.aprilTagRecognition,
                                                    robotProfile, teamPropPos, drive, isRed ));
        goToDropBoard();
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, parkingPose1, true));
        taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, parkingPose2, true));
        return taskList;
    }
    TrajectorySequence firstTraj1=null, firstTraj2=null, firstTraj3=null;
    private void buildLeftTrajectory(){
        TrajectorySequenceBuilder wpBuilder = drive.trajectorySequenceBuilder(team_prop_pos_traj.end());
                if(passThrough.equals("MIDDLE")) {
                    wpBuilder.splineToLinearHeading(wp1,Math.toRadians(wp1.getHeading()));
                    wpBuilder.splineToLinearHeading(wp2, Math.toRadians(wp2.getHeading()));
                }else {
                    wpBuilder.splineToLinearHeading(wp1,Math.toRadians(wp1.getHeading()));
                    wpBuilder.splineToLinearHeading(wp2, Math.toRadians(wp2.getHeading()));
                }
        TrajectorySequence wpTraj1= wpBuilder.build();
        taskList.add(new SplineMoveTask(drive, wpTraj1));

        TrajectorySequenceBuilder wpBuilder2 = drive.trajectorySequenceBuilder(wpTraj1.end());
            if(passThrough.equals("MIDDLE")) {
                wpBuilder2.lineTo(wp3.vec());
            }else {
                wpBuilder2.splineToLinearHeading(wp3, Math.toRadians(wp3.getHeading()));
            }
        TrajectorySequence  wpTraj2  = wpBuilder2.build();
        taskList.add(new SplineMoveTask(drive, wpTraj2));

        TrajectorySequenceBuilder builder3 = drive.trajectorySequenceBuilder(wpTraj2.end())
                .lineTo(wp4.vec());
                if(wp5 != null) {
                    builder3.lineTo(wp5.vec());
                }
                if(wp6 != null) {
                    builder3.lineTo(wp6.vec());
                }
                builder3.lineTo(aprilTagPt.vec());
        TrajectorySequence wpTraj3 = builder3.build();
        taskList.add(new SplineMoveTask(drive, wpTraj3));
    }
    private void buildCenterTrajectory(){
        TrajectorySequenceBuilder wpBuilder = drive.trajectorySequenceBuilder(team_prop_pos_traj.end());
        if(passThrough.equals("WALL") ) {
            wpBuilder.splineToLinearHeading(wp1, Math.toRadians(wp1.getHeading()));
            if(isRed) {
                wpBuilder.splineToLinearHeading(wp2, Math.toRadians(wp2.getHeading()));
            }else{
                wpBuilder.lineTo(wp2.vec());
            }
            wpBuilder.lineTo(wp3.vec());
        }else{
            wpBuilder.lineTo(wp1.vec())
                    .lineTo(wp2.vec())
                    .splineToLinearHeading(wp3, Math.toRadians(wp3.getHeading()));
        }
        TrajectorySequence wpTraj1= wpBuilder.build();
        taskList.add(new SplineMoveTask(drive, wpTraj1));

        TrajectorySequenceBuilder wpBuilder2 = drive.trajectorySequenceBuilder(wpTraj1.end());
        if(isRed && passThrough.equals("MIDDLE")){
            wpBuilder2.splineToLinearHeading(wp4,Math.toRadians(wp4.getHeading()));
        }else {
            wpBuilder2.lineTo(wp4.vec());
        }
        if(wp5 != null) {
            wpBuilder2.lineTo(wp5.vec());
        }
        if(wp6 != null) {
            wpBuilder2.lineTo(wp6.vec());
        }
        if(wp7 != null) {
            wpBuilder2.lineTo(wp7.vec());
        }
        wpBuilder2.lineTo(aprilTagPt.vec());
        TrajectorySequence wpTraj2 = wpBuilder2.build();
        taskList.add(new SplineMoveTask(drive, wpTraj2));
    }
    private void buildRightTrajectory(){
        TrajectorySequenceBuilder wpBuilder = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                .lineTo(wp1.vec());
        if(passThrough.equals("MIDDLE")) {
            wpBuilder.lineTo(wp2.vec());
            wpBuilder.splineToLinearHeading(wp3, Math.toRadians(wp3.getHeading()));
        }else{
            wpBuilder.splineToLinearHeading(wp2, Math.toRadians(wp2.getHeading()));
            wpBuilder.lineTo(wp3.vec());
        }
        TrajectorySequence wpTraj1= wpBuilder.build();
        taskList.add(new SplineMoveTask(drive, wpTraj1));

        TrajectorySequenceBuilder wpBuilder2 = drive.trajectorySequenceBuilder(wpTraj1.end());
        wpBuilder2.lineTo(wp4.vec());
        if(wp5 != null) {
            wpBuilder2.lineTo(wp5.vec());
        }
        if(wp6 != null) {
            wpBuilder2.lineTo(wp6.vec());
        }
        wpBuilder2.lineTo(aprilTagPt.vec());
        TrajectorySequence wpTraj3 = wpBuilder2.build();
        taskList.add(new SplineMoveTask(drive, wpTraj3));
    }

    void goToDropBoard(){
        taskList.add(new RobotSleep(1000));
        taskList.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.CLOSE));
        if(teamPropPos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT)) {
            taskList.add(new PixelUpTask(robotHardware, true, robotProfile.hardwareSpec.liftOutAuto));
        }else{
            taskList.add(new PixelUpTask(robotHardware, false, robotProfile.hardwareSpec.liftOutAuto));
        }
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
