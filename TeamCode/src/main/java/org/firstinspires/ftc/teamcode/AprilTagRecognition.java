package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagRecognition {
    AprilTagProcessor aprilTag;
    boolean USE_WEBCAM = true;
    HardwareMap hardwareMap;

    VisionPortal visionPortal;
    final double DESIRED_DISTANCE = 1; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.

    public AprilTagRecognition(boolean use_webcam, HardwareMap hardwareMap){
        this.USE_WEBCAM = use_webcam;
        this.hardwareMap = hardwareMap;
    }

    public void initAprilTag(){
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();
                aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
//        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public List getAprilTagResult(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Logger.logFile("# AprilTags Detected"+currentDetections.size());
        return currentDetections;
    }

    //Autonomous mode
    public void goToAprilTag(int id){
        this.DESIRED_TAG_ID = id;
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        AprilTagDetection tagData = null;
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        double  rangeDiff       = 0;
        double  bearingDiff     = 0;
        double  yawDiff         = 0;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Logger.logFile("# AprilTags Detected"+currentDetections.size());
        for(AprilTagDetection tag : currentDetections){
            if(tag.metadata !=null) {
                Logger.logFile("this tag id = " + tag.id);
                if (tag.id == id) {
                    targetFound = true;
                    tagData = tag;
                    //todo close the portal as camera is no longer needed? visionPortal.close();
                    break;
                } else {  //skip this id
                    Logger.logFile("skip id:" + tag.id);
                }
            }else {
                Logger.logFile("There is no tag detected");
            }
        }

//        Pitch: Rotation of AprilTag around the X axis. A pitch value of zero implies that the camera is directly in front of the Tag, as viewed from the side.
//        Roll: Rotation of AprilTag around the Y axis. A roll value of zero implies that the Tag image is alligned squarely and upright, when viewed in the camera image frame.
//        Yaw: Rotation of AprilTag around the Z axis. A yaw value of zero implies that the camera is directly in front of the Tag, as viewed from above
        if(targetFound){
            Logger.logFile("range, bearing, yaw (RBY) are => " + tagData.ftcPose.range + ", " + tagData.ftcPose.bearing + ", " + tagData.ftcPose.yaw);
            Logger.logFile("x, y, z (XYZ) direction from the camera => " + tagData.ftcPose.x + ", " + tagData.ftcPose.y + ", " + tagData.ftcPose.z);
            Logger.logFile("pitch, roll, yaw (PRY) from the camera to tag => " + tagData.ftcPose.pitch + ", " + tagData.ftcPose.roll + ", " + tagData.ftcPose.yaw);
            rangeDiff = tagData.ftcPose.range - this.DESIRED_DISTANCE;
            bearingDiff = tagData.ftcPose.bearing;
            yawDiff = tagData.ftcPose.yaw;
        }
    }

    public void gotoAprilTaginMotion() throws InterruptedException {
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

    }

    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Logger.logFile("Camera" + "Waiting");
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                Thread.sleep(20);
            }

            Logger.logFile("Camera Ready");
            Logger.flushToFile();
        }

        // Set camera controls unless we are stopping.
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            Thread.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            Thread.sleep(20);
        }
    }
}
