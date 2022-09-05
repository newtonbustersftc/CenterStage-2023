package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * SplineMoveTask
 * Created by Gavin Fountain
 */

public class SplineMoveTask implements RobotControl {

    NBMecanumDrive drive;
    Trajectory trajectory;
    TrajectorySequence trajectorySequence;
    Pose2d targetPose;
    TrajectoryVelocityConstraint velocityConstraint;

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

    public String toString() {
        return "SplineMove " + trajectory.start() + " -> " + trajectory.end() ;
    }

    public boolean isDone(){
        return !drive.isBusy();
    }

    public void prepare(){
        if (targetPose!=null) {
            Pose2d currPose = drive.getPoseEstimate();
            double ang = Math.atan2(targetPose.getX() - currPose.getX(), targetPose.getY() - currPose.getY());
            boolean forward = Math.abs(currPose.getHeading() - ang) < Math.PI / 2;
            trajectory = drive.trajectoryBuilder(currPose, !forward)
                        .splineToSplineHeading(targetPose, targetPose.getHeading()).build();
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
