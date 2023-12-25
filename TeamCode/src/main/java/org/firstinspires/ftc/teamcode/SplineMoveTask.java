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
    TrajectorySequence trajectorySequence;
    Pose2dRef targetPose;
    boolean isLineTo = false;

    public SplineMoveTask(NBMecanumDrive drive, Trajectory trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
        targetPose = null;
    }

    public SplineMoveTask(NBMecanumDrive drive, Pose2dRef targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
    }

    public SplineMoveTask(NBMecanumDrive drive, TrajectorySequence trajectorySequence) {
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;
        this.targetPose = null;
    }

    //AprilTag
    public SplineMoveTask(NBMecanumDrive drive, Pose2dRef targetPose, boolean isLineTo){
        this.drive = drive;
        this.targetPose = targetPose;
        this.isLineTo = isLineTo;
    }

    public String toString() {
        if (trajectory != null) {
            return "SplineMove " + trajectory.start() + " -> " + trajectory.end();
        } else if (trajectorySequence != null) {
            return "SplineMove " + trajectorySequence.start() + " -> " + trajectorySequence.end();
        } else if (targetPose != null) {
            return "SplineMove targetPose x:" + targetPose.getPose2d().getX()+ ", y: " + targetPose.getPose2d().getY() +", heading:" +
                    targetPose.getPose2d().getHeading();
        }else{
            return "Invalid task.... trajectory, trajectorySequence, or trajectoryTag should be not null..";
        }
    }

    public boolean isDone(){
        return !drive.isBusy();
    }

    public void prepare(){
        if (targetPose!=null) {
            Pose2d currPose = drive.getPoseEstimate();
            if (isLineTo) {
                trajectory = drive.trajectoryBuilder(currPose).lineTo(targetPose.getPose2d().vec()).build();
            }
            else {
                double ang = Math.atan2(targetPose.getPose2d().getX() - currPose.getX(), targetPose.getPose2d().getY() - currPose.getY());
                boolean forward = Math.abs(currPose.getHeading() - ang) < Math.PI / 2;
                trajectory = drive.trajectoryBuilder(currPose, !forward)
                        .splineToSplineHeading(targetPose.getPose2d(), targetPose.getPose2d().getHeading()).build();
            }
            drive.followTrajectoryAsync(trajectory);
        }else if(trajectorySequence!=null) {
            drive.followTrajectorySequenceAsync(trajectorySequence);
        }else if(trajectory !=null) {
            drive.followTrajectoryAsync(trajectory);
        }
    }

    public void execute() {
        drive.update();
    }

    public void cleanUp(){

    }

}
