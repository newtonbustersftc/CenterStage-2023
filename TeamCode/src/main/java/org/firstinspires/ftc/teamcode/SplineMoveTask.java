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
    TrajectorySequence trajectorySequence, trajectoryTag, trajectoryParking;
    Pose2d targetPose;
    TrajectoryVelocityConstraint velocityConstraint;
    RobotHardware robotHardware;
    AprilTagDetection desiredTag;
    boolean isRed;
    int WALL_LEFT = 26, WALL_CENTER=18, WALL_RIGHT=10, forward=8;

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
    public SplineMoveTask(NBMecanumDrive drive, RobotHardware robotHardware, boolean isRed){
        this.drive = drive;
        this.robotHardware = robotHardware;
        this.targetPose = null;
        this.isRed = isRed;
        Logger.logFile("SplineMoveTask constructor");
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
        Logger.logFile("SplineMoveTask....1");
        if (targetPose!=null) {
            Logger.logFile("SplineMoveTask....2");
            Pose2d currPose = drive.getPoseEstimate();
            double ang = Math.atan2(targetPose.getX() - currPose.getX(), targetPose.getY() - currPose.getY());
            boolean forward = Math.abs(currPose.getHeading() - ang) < Math.PI / 2;
            trajectory = drive.trajectoryBuilder(currPose, !forward)
                        .splineToSplineHeading(targetPose, targetPose.getHeading()).build();
            drive.followTrajectoryAsync(trajectory);
        }else if(trajectorySequence!=null) {
            Logger.logFile("SplineMoveTask....3");
            drive.followTrajectorySequenceAsync(trajectorySequence);
        }else if(trajectory !=null) {
            Logger.logFile("SplineMoveTask....4");
            drive.followTrajectoryAsync(trajectory);
        }else if(robotHardware != null ){ //this must be the detected AprilTag
            Logger.logFile("SplineMoveTask....5");
            Pose2d currentPose = drive.getPoseEstimate();
            Logger.logFile("currentPose x:"+currentPose.getX() + " y:"+currentPose.getY());
            desiredTag = robotHardware.getDesiredAprilTag();
            double heading=desiredTag.ftcPose.yaw, x=desiredTag.ftcPose.x, y=desiredTag.ftcPose.y;
            double reCalculatedX, reCalculatedY;

            //adjust Y because of the camera location,
            if(x < 0){
                reCalculatedY = currentPose.getY() + Math.abs(x);
            }else{
                reCalculatedY = currentPose.getY() - x;
            }

            //adjust X to back up a bit to have enough space for lift
            reCalculatedX = currentPose.getX()  + desiredTag.ftcPose.range * Math.cos(Math.toRadians(desiredTag.ftcPose.bearing));
            Logger.logFile("reCalculatedX="+reCalculatedX);
            Logger.logFile("reCalculatedY="+reCalculatedY);
            Logger.logFile("heading="+heading);
            Logger.flushToFile();

            if(isRed){
                targetPose = new Pose2d(reCalculatedX - 1.5, reCalculatedY , heading);
            }else {
                targetPose = new Pose2d(reCalculatedX - 1.5, reCalculatedY + 6, heading);
            }
            trajectoryTag = drive.trajectorySequenceBuilder(currentPose)
                        .splineTo(targetPose.vec(), Math.toRadians(heading))
                        .build();
            drive.followTrajectorySequenceAsync(trajectoryTag);
//            drive.getLocalizer().setPoseEstimate(targetPose);
        }
    }

    public void execute() {
        drive.update();
    }

    public void cleanUp(){

    }

}
