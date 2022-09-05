package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

public class InstantaneousPostionTrajectoryTask implements RobotControl {
    RobotHardware robotHardware;
    NBMecanumDrive drive;
    Pose2d nextPos;
    boolean isBackward;
    ArrayList<Pose2d> poses;
    ArrayList<TrajectoryVelocityConstraint> poseSpeed;

    public InstantaneousPostionTrajectoryTask(RobotHardware robotHardware, Pose2d nextPos, boolean isBackward){
        this.robotHardware = robotHardware;
        this.drive = robotHardware.mecanumDrive;
        this.nextPos = nextPos;
        this.isBackward = isBackward;
    }

    public InstantaneousPostionTrajectoryTask(RobotHardware robotHardware, ArrayList<Pose2d> poses, ArrayList<TrajectoryVelocityConstraint> poseSpeed, boolean isBackward){
        this.robotHardware = robotHardware;
        this.drive = robotHardware.mecanumDrive;
        this.poses = poses;
        this.isBackward = isBackward;
        this.poseSpeed = poseSpeed;
    }



    @Override
    public void prepare() {
        TrajectorySequenceBuilder trajectorySequenceBuilder ;
        TrajectorySequence trajectorySequence;
        TrajectoryAccelerationConstraint accConstraint = SampleMecanumDrive.getAccelerationConstraint((40));
        TrajectoryVelocityConstraint velConstraints = SampleMecanumDrive.getVelocityConstraint(25, 25, 10.25);
        TrajectoryVelocityConstraint fastVelConstraints = SampleMecanumDrive.getVelocityConstraint(50, 50, 10.25);
        TrajectoryVelocityConstraint slowVelConstraints = SampleMecanumDrive.getVelocityConstraint(10, 10, 10.25);

        Pose2d curPos = robotHardware.getLocalizer().getPoseEstimate();
        Logger.logFile("pick up pose in InstantaneousPositionTrajectoryTask: " + curPos);
        if(isBackward) {
            if(poses == null){  //only nextPos to handle
                trajectorySequence = drive.trajectorySequenceBuilder(curPos)
                        .setReversed(true)
                        .splineTo(nextPos.vec(), nextPos.getHeading() + Math.PI, velConstraints, accConstraint)
                        .build();
            }else {
                trajectorySequenceBuilder = drive.trajectorySequenceBuilder(curPos);
                trajectorySequenceBuilder.setReversed(true);
                for(int i = 0; i<poses.size()-1; i++){
                    Pose2d pose = poses.get(i);
                    TrajectoryVelocityConstraint myVelocityConstraint = poseSpeed.get(i);
                    trajectorySequenceBuilder.splineTo(pose.vec(), pose.getHeading()+Math.PI, myVelocityConstraint, accConstraint );
               }
                Pose2d pose = poses.get(poses.size()-1);
                trajectorySequenceBuilder.splineToLinearHeading(pose
                        , pose.getHeading()+Math.PI, poseSpeed.get(poses.size()-1), accConstraint );
                trajectorySequence = trajectorySequenceBuilder.build();
            }
        }else{
            if(poses == null){
                trajectorySequence = drive.trajectorySequenceBuilder(curPos)
                        .splineTo(nextPos.vec(), nextPos.getHeading(), velConstraints, accConstraint)
                        .build();
            }else{
                trajectorySequenceBuilder = drive.trajectorySequenceBuilder(curPos);
                for(int i = 0; i<poses.size()-1; i++){
                    Pose2d pose = poses.get(i);
                    TrajectoryVelocityConstraint myVelocityConstraint = poseSpeed.get(i);
                    trajectorySequenceBuilder.splineTo(pose.vec(), pose.getHeading()+Math.PI, fastVelConstraints, accConstraint );
                }
                Pose2d pose = poses.get(poses.size()-1);
                trajectorySequenceBuilder.splineToLinearHeading(pose, pose.getHeading()+Math.PI, poseSpeed.get(poses.size()-1), accConstraint );
                trajectorySequence = trajectorySequenceBuilder.build();
            }
        }
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void cleanUp() {

    }

    @Override
    public boolean isDone() {
        return !drive.isBusy();
    }
}
