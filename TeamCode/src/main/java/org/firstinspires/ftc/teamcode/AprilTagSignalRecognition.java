package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

//@TeleOp
public class AprilTagSignalRecognition  {

        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        // Tag ID 1,2,3 from the 36h11 family
        int LEFT = 1;
        int MIDDLE = 2;
        int RIGHT = 3;

        AprilTagDetection tagOfInterest = null;
        RobotVision rVision = null;

        public AprilTagSignalRecognition(RobotVision rVision){
            this.rVision = rVision;
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        }

        public void startRecognition(){
            this.rVision.initWebCam("Webcam 1", false);
//            camera = this.rVision.cameraMap.get("Webcam 1");
            try {
                Thread.sleep(2000);
            }
            catch (Exception e) {}
            rVision.startWebcam("Webcam 1", aprilTagDetectionPipeline);
        }

        public int getRecognitionResult() {
//            camera.setPipeline(aprilTagDetectionPipeline);
//            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
//                }
//
//                @Override
//                public void onError(int errorCode) {
//                }
//            });

            /* The INIT-loop:
             * This REPLACES waitForStart!
             */
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    Logger.logFile("Tag of interest is in sight!\n\nLocation data:" + tagOfInterest);
                } else {
                    Logger.logFile("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        Logger.logFile("(The tag has never been seen)");
                    } else {
                        Logger.logFile("\nBut we HAVE seen the tag before; last seen at:" + tagOfInterest);
                    }
                }
            } else {
                Logger.logFile("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    Logger.logFile("(The tag has never been seen)");
                } else {
                    Logger.logFile("\nBut we HAVE seen the tag before; last seen at:" + tagOfInterest);
                }
            }

            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            /* Update the telemetry */
            if (tagOfInterest != null) {
                Logger.logFile("Tag snapshot:\n" + tagOfInterest);
            } else {
                Logger.logFile("No tag snapshot available, it was never sighted during the init loop :(");
            }

            /* Actually do something useful */
            if (tagOfInterest == null) {
                return 0;
            }else if(tagOfInterest.id == LEFT){
                return LEFT;
            }else if(tagOfInterest.id == MIDDLE){
                return MIDDLE;
            }else{
                return RIGHT;
            }

        }

        void tagToLogger(AprilTagDetection detection)
        {
            Logger.logFile(String.format("\nDetected tag ID=%d", detection.id));
            Logger.logFile(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
            Logger.logFile(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
            Logger.logFile(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
            Logger.logFile(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            Logger.logFile(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            Logger.logFile(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }
    }

