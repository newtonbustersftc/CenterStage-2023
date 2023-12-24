package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

public class AprilTagDetectionTask implements RobotControl {
    private  int desiredTagId = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound=false;
    RobotHardware robotHardware;
    PixelBoardVision aprilTagRecognition;
    String teamPropPos;
    NBMecanumDrive drive;
    boolean isRed;
    double desiredAprilTagX, desiredAprilTagY, totalX, totalY, totalHeading, sizeOfDetectionIDList, startTime;

    public AprilTagDetectionTask(RobotHardware robotHardware, PixelBoardVision aprilTagRecognition, RobotProfile profile,
                                 String teamPropPos, NBMecanumDrive drive, boolean isRed){
        this.robotHardware = robotHardware;
        this.aprilTagRecognition = aprilTagRecognition;
        this.teamPropPos = teamPropPos;
        this.drive = drive;
        this.isRed = isRed;
        String color = isRed ? "RED" : "BLUE";
        desiredAprilTagX = profile.getProfilePose("DROPBOARD_APRILTAG_"+color+"_"+ teamPropPos).getX();
        desiredAprilTagY = profile.getProfilePose("DROPBOARD_APRILTAG_"+color+"_"+ teamPropPos).getY();
    }

    @Override
    public void prepare()  {
        startTime = System.currentTimeMillis();
        if(this.teamPropPos.equals("LEFT")){
           desiredTagId = isRed ? 4 : 1;
        } else if(this.teamPropPos.equals("CENTER")){
           desiredTagId = isRed ? 5 : 2;
        } else if(this.teamPropPos.equals("RIGHT")){
           desiredTagId = isRed ? 6 : 3;
        }
        robotHardware.desiredAprilTagId = desiredTagId;   // need by PixelBoardVision
        Logger.logFile("In AprilTagDetectionTask -> desired_tag_id="+ desiredTagId);
        Logger.flushToFile();
    }

    @Override
    public void execute() {
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = this.aprilTagRecognition.aprilTag.getDetections();
        if (currentDetections==null) {
            Logger.logFile("AprilTag detections is null");
        }
        else {
            sizeOfDetectionIDList = currentDetections.size();
            Logger.logFile("AprilTag detected count: " + sizeOfDetectionIDList);
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    checkID(detection);
                }
            }
            if (totalX > 0) {
                targetFound = true;
                setUpdatePose();
                goToDesiredTag();
            }
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
        robotHardware.resetDriveAndEncoders();
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

        Logger.logFile("desired ID = "+ desiredTagId);
        Logger.logFile("detected ID = "+detection.id);
        Logger.logFile("ftcPose.x="+detection.ftcPose.x);
        Logger.logFile("ftcPose.y="+detection.ftcPose.y);
        Logger.logFile("ftcPose.yaw="+ detection.ftcPose.yaw);
        Logger.logFile("ftcPose.bearing="+detection.ftcPose.bearing);
        Logger.logFile("Y="+thisY +", calculatedY="+detection.ftcPose.range * Math.cos(Math.toRadians(detection.ftcPose.bearing)));
        Logger.logFile("range=" + detection.ftcPose.range);
        Logger.logFile("current robot heading = " + currentPose.getHeading());
        Logger.logFile("desiredAprilTagX="+desiredAprilTagX);
        Logger.logFile("desiredAprilTagY="+desiredAprilTagY);

        thisY = idYCoordinations[detection.id-1] + thisX - centerOFRobotY;
        thisX = dropBoardX - centerOfRobotX - detection.ftcPose.y;
        updatedPose = new Pose2d(thisX, thisY, -Math.toRadians(heading));
        Logger.logFile("updatedPose.x="+updatedPose.getX() +", y="+updatedPose.getY() + ", " + updatedPose.getHeading());
        return updatedPose;
    }
    private void goToDesiredTag(){
        Logger.logFile("*********in go to desired tag *********");
        Pose2d currentPose = drive.getPoseEstimate();

        Logger.logFile("desiredAprilTagX="+desiredAprilTagX);
        Logger.logFile("desiredAprilTagY="+desiredAprilTagY);
        Pose2d targetPose = new Pose2d(desiredAprilTagX, desiredAprilTagY, 0);

        TrajectorySequence trajectoryTag = drive.trajectorySequenceBuilder(currentPose)
                .lineToLinearHeading(targetPose)
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
