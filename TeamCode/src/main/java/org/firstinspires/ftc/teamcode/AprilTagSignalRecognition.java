package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.List;

//@TeleOp
public class AprilTagSignalRecognition  {

        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        //the AprilTags are provide by FTC
        AprilTagProcessor aprilTag;
        VisionPortal.Builder builder;
        VisionPortal visionPortal;

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

        public AprilTagSignalRecognition(RobotHardware hardware){
            Logger.logFile("init");
            aprilTag = new AprilTagProcessor.Builder().build();
            builder = new VisionPortal.Builder();
            builder.setCamera(hardware.hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.addProcessor(aprilTag);
            visionPortal = builder.build();
        }

        public void startRecognition(){
            this.rVision.initWebCam("Webcam 1", false);
            rVision.startWebcam("Webcam 1", aprilTagDetectionPipeline);
        }

        public void stopRecognition() {
            rVision.stopWebcam("Webcam 1");
            visionPortal.stopStreaming();
        }

        public int getRecognitionResult() {
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
            }
            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            /* Actually do something useful */
            if (tagOfInterest == null) {
                return 0;
            }else if(tagOfInterest.id == LEFT){
                return LEFT;
            }else if(tagOfInterest.id == MIDDLE){
                return MIDDLE;
            }else if(tagOfInterest.id == RIGHT){
                return RIGHT;
            }else{
                return 0;
            }
        }

        public List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> getDetectionList(){
            return aprilTag.getDetections();
        }
    }

