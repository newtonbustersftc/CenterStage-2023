package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

public class AprilTagDetectionTask implements RobotControl {
    private  int  DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound=false;
    RobotHardware robotHardware;
    AprilTagRecognition aprilTagRecognition;
    RobotCVProcessor.TEAM_PROP_POS team_prop_pos;
    NBMecanumDrive drive;
    boolean isRed;
    double desiredAprilTagX, desiredAprilTagY, totalX, totalY, totalHeading, sizeOfDetectionIDList, startTime;

    public AprilTagDetectionTask(RobotHardware robotHardware, AprilTagRecognition aprilTagRecognition, RobotProfile profile,
                                 RobotCVProcessor.TEAM_PROP_POS team_prop_pos, NBMecanumDrive drive, boolean isRed){
        this.robotHardware = robotHardware;
        this.aprilTagRecognition = aprilTagRecognition;
        this.team_prop_pos = team_prop_pos;
        this.drive = drive;
        this.isRed = isRed;
        String color = isRed ? "RED" : "BLUE";
        desiredAprilTagX = profile.getProfilePose("DROPBOARD_APRILTAG_"+color+"_"+team_prop_pos.toString()).getX();
        desiredAprilTagY = profile.getProfilePose("DROPBOARD_APRILTAG_"+color+"_"+team_prop_pos.toString()).getY();
    }

    @Override
    public void prepare()  {
        startTime = System.currentTimeMillis();
        Logger.logFile("in AprilTagTask prepare()");
       if(this.team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.LEFT)){
           DESIRED_TAG_ID = isRed ? 4 : 1;
       }else if(this.team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.CENTER )){
           DESIRED_TAG_ID = isRed ? 5 : 2;
       }else if(this.team_prop_pos.equals(RobotCVProcessor.TEAM_PROP_POS.RIGHT)){
           DESIRED_TAG_ID = isRed ? 6 : 3;
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
        sizeOfDetectionIDList = currentDetections.size();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null ){
                checkID(detection);
            }
        }
        if(totalX>0) {
            targetFound = true;
            setUpdatePose();
            goToDesiredTag();
        }
    }

    private void checkID(AprilTagDetection detection){
        Logger.logFile("*********in checkID*********");
        Pose2d pose = getLocalizerPose(detection);
        totalX += pose.getX();
        totalY += pose.getY();
        totalHeading += pose.getHeading();
        Logger.logFile("totalX:"+totalX);
        Logger.logFile("totalY:"+totalY);
        Logger.logFile("totalHeading:"+totalHeading);
    }
    private void setUpdatePose(){
        Logger.logFile("*********in setUpdate pose*********");
        Pose2d updatedPose = new Pose2d(totalX/sizeOfDetectionIDList, totalY/sizeOfDetectionIDList,
                                            totalHeading/sizeOfDetectionIDList);
        drive.getLocalizer().setPoseEstimate(updatedPose);
        Logger.logFile("averagedPoseX:"+updatedPose.getX());
        Logger.logFile("averagedPoseY:"+updatedPose.getY());
        Logger.logFile("averagedPoseHeading:"+updatedPose.getHeading());
    }
    private Pose2d getLocalizerPose(AprilTagDetection detection){
        Logger.logFile("*********in calculate current pose and update*********");
        int[] idYCoordinations={41,35,29,-29,-35,-41};
        double dropBoardX = 61, centerOfRobotX = 8.5, centerOFRobotY = 4; //centerOFRobotY=6

        double thisX=detection.ftcPose.x, thisY=detection.ftcPose.y, heading = detection.ftcPose.yaw;
        Pose2d updatedPose=null, currentPose = drive.getPoseEstimate();

        Logger.logFile("desired ID = "+DESIRED_TAG_ID);
        Logger.logFile("detected ID = "+detection.id);
        Logger.logFile("ftcPose.x="+detection.ftcPose.x);
        Logger.logFile("ftcPose.y="+detection.ftcPose.y);
        Logger.logFile("ftcPose.yaw="+ detection.ftcPose.yaw);
        Logger.logFile("ftcPose.bearing="+detection.ftcPose.bearing);
        Logger.logFile("Y="+thisY +", calculatedY="+detection.ftcPose.range * Math.cos(Math.toRadians(detection.ftcPose.bearing)));
        Logger.logFile("range=" + detection.ftcPose.range);
        Logger.logFile("current robot heading = " + currentPose.getHeading());

        thisY = idYCoordinations[detection.id-1] + thisX - centerOFRobotY;
        thisX = dropBoardX - centerOfRobotX - detection.ftcPose.y;
        updatedPose = new Pose2d(thisX, thisY, -Math.toRadians(heading));
        Logger.logFile("updatedPose.x="+updatedPose.getX() +", y="+updatedPose.getY() + ", " + updatedPose.getHeading());
        return updatedPose;
    }
    private void goToDesiredTag(){
        Logger.logFile("*********in go to desired tag *********");
        Pose2d currentPose = drive.getPoseEstimate();
        Logger.logFile("the currentPose.x="+currentPose.getX());
        Logger.logFile("the currentPose.y="+currentPose.getY());
        Logger.logFile("the currentPose.heading="+currentPose.getHeading());
        Logger.logFile("desiredAprilTagX="+desiredAprilTagX);
        Logger.logFile("desiredAprilTagY="+desiredAprilTagY);
        Pose2d targetPose = new Pose2d(desiredAprilTagX, desiredAprilTagY, 0);

        TrajectorySequence trajectoryTag = drive.trajectorySequenceBuilder(currentPose)
//                .splineTo(targetPose.vec(), targetPose.getHeading())
                .lineTo(targetPose.vec())
                .build();
        drive.followTrajectorySequence(trajectoryTag);
        Logger.logFile("done with trajectoryTag");
    }

    @Override
    public void cleanUp() {
        targetFound = false;
        aprilTagRecognition.visionPortal.close();
    }

    @Override
    public boolean isDone() {
        return targetFound;
    }
}
