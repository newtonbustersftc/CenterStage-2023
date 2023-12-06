package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * SplineMoveTask
 * Created by Gavin Fountain
 */

public class SplineMoveTask implements RobotControl {

    NBMecanumDrive drive;
    Trajectory trajectory;
    TrajectorySequence trajectorySequence, trajectoryTag;
    Pose2d targetPose;
    TrajectoryVelocityConstraint velocityConstraint;
    RobotHardware robotHardware;
    AprilTagDetection desiredTag;
    boolean isRed;
    int parkingDistance;
    boolean isLineTo = false;
    Pose2d parkingPose;

    public SplineMoveTask(NBMecanumDrive drive, Trajectory trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
        targetPose = null;
    }

    public SplineMoveTask(NBMecanumDrive drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
    }

    public SplineMoveTask(NBMecanumDrive drive, TrajectorySequence trajectorySequence) {
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;
        this.targetPose = null;
    }

    //AprilTag
    public SplineMoveTask(NBMecanumDrive drive, RobotHardware robotHardware, Pose2d parkingPose){
        this.drive = drive;
        this.robotHardware = robotHardware;
        this.parkingPose = parkingPose;
    }

    public String toString() {
        if (trajectory != null) {
            return "SplineMove " + trajectory.start() + " -> " + trajectory.end();
        } else if (trajectorySequence != null) {
            return "SplineMove " + trajectorySequence.start() + " -> " + trajectorySequence.end();
        } else if (trajectoryTag != null) {
            return "SplineMove " + trajectoryTag.start() + " -> " + trajectoryTag.end();
        } else if (robotHardware != null){
            return "SplineMove - going to create new trajectory based on AprilTag." ;
        }else{
            return "the robot should not come to here.... trajectory, trajectorySequence, or trajectoryTag should be not null..";
        }
    }

    public boolean isDone(){
        return !drive.isBusy();
    }

    public void prepare(){
        if (targetPose!=null) {
            Pose2d currPose = drive.getPoseEstimate();
            if (isLineTo) {
                trajectory = drive.trajectoryBuilder(currPose).lineTo(targetPose.vec()).build();
            }
            else {
                double ang = Math.atan2(targetPose.getX() - currPose.getX(), targetPose.getY() - currPose.getY());
                boolean forward = Math.abs(currPose.getHeading() - ang) < Math.PI / 2;
                trajectory = drive.trajectoryBuilder(currPose, !forward)
                        .splineToSplineHeading(targetPose, targetPose.getHeading()).build();
            }
            drive.followTrajectoryAsync(trajectory);
        }else if(trajectorySequence!=null) {
            drive.followTrajectorySequenceAsync(trajectorySequence);
        }else if(trajectory !=null) {
            drive.followTrajectoryAsync(trajectory);
        }else if(robotHardware != null ){ //this must be the detected AprilTag
            Pose2d currentPose = drive.getPoseEstimate();
            Logger.logFile("currentPose x:"+currentPose.getX() + " y:"+currentPose.getY());
            Logger.logFile("parkingDistance="+parkingDistance);

            trajectoryTag = drive.trajectorySequenceBuilder(currentPose)
                        .back(2)
                        .lineTo(parkingPose.vec())
                        .build();
            drive.followTrajectorySequenceAsync(trajectoryTag);
        }
    }

    public void execute() {
        drive.update();
    }

    public void cleanUp(){

    }

}
