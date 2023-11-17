package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDetectionTask implements RobotControl {
    final double DESIRED_DISTANCE = 1; //  this is how close the camera should get to the target (inches)

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static  int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
              // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound=false;
    RobotHardware robotHardware;
    AprilTagRecognition aprilTagRecognition;
    RobotCVProcessor.TEAM_PROP_POS team_prop_pos;
    NBMecanumDrive drive;

    public AprilTagDetectionTask(RobotHardware robotHardware, AprilTagRecognition aprilTagRecognition,
                                 RobotCVProcessor.TEAM_PROP_POS team_prop_pos, NBMecanumDrive drive){
        this.robotHardware = robotHardware;
        this.aprilTagRecognition = aprilTagRecognition;
        this.team_prop_pos = team_prop_pos;
        this.drive = drive;
    }
    @Override
    public void prepare()  {
       if(this.team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT)){
           DESIRED_TAG_ID = 1;
       }else if(this.team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.CENTER )){
           DESIRED_TAG_ID = 2;
       }else if(this.team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT)){
           DESIRED_TAG_ID = 3;
       }

       Logger.logFile("In AprilTagDetectionTask -> desired_tag_id="+DESIRED_TAG_ID);
       Logger.flushToFile();
    }

    @Override
    public void execute() {
        Logger.logFile("aprilTagRecognition.aprilTag.getDetections.size:"+this.aprilTagRecognition.aprilTag.getDetections().size());
        Logger.logFile("aprilTagRecognition size:" + this.aprilTagRecognition.getAprilTagResult().size());
        Logger.logFile("aprilTagRecognition camera state:" + this.aprilTagRecognition.visionPortal.getCameraState().toString());

// Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = this.aprilTagRecognition.aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            Logger.logFile("am I here....1");
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                Logger.logFile("am I here....2");
                //  Check to see if we want to track towards this tag.
                if (detection.id == DESIRED_TAG_ID) {
                    Logger.logFile("am I here....3");
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    Logger.logFile("In AprilTagDetectionTask, desiredTag is found id = "+ desiredTag.id);
                    Logger.flushToFile();
                    this.robotHardware.storeDesiredAprilTag(desiredTag);
                    break;  // don't look any further.
                }
            }
        }
    }

    @Override
    public void cleanUp() {
        targetFound = false;
//        Pose2d currentPose = drive.getPoseEstimate();
//        double heading=desiredTag.ftcPose.yaw, x=desiredTag.ftcPose.x, y=desiredTag.ftcPose.y;
//        double reCalculatedX, reCalculatedY;
//        if(x < 0){
//            reCalculatedY = currentPose.getY() + Math.abs(x);
//        }else{
//            reCalculatedY = currentPose.getY() - x;
//        }
//
//        reCalculatedX = currentPose.getX() + desiredTag.ftcPose.range * Math.cos(heading);
//        Logger.logFile("reCalculatedX="+reCalculatedX);
//        Logger.logFile("reCalculatedY="+reCalculatedY);
//        Logger.logFile("heading="+heading);
//
//        Pose2d targetPose = new Pose2d(reCalculatedX, reCalculatedY, heading);
//        Trajectory trajectoryTag = drive.trajectoryBuilder(currentPose)
//                .splineTo(targetPose.vec(), targetPose.getHeading())
//                .build();
//        drive.followTrajectoryAsync(trajectoryTag);
//        aprilTagRecognition.stopStream();
        aprilTagRecognition.visionPortal.close();
    }

    @Override
    public boolean isDone() {
        if(targetFound) {
            return true;
        }else{
            return false;
        }
    }

}
