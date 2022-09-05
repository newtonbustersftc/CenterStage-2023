package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 2020.11.05
 claire
 **/

public class RobotVision {
    public enum AutonomousGoal {LEFT, MIDDLE, RIGHT, NONE};
    private List<VuforiaTrackable> allTrackables;
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null ;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    OpenCvWebcam rearCamera = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private static final String VUFORIA_KEY = "AYqWjPr/////AAABmVMnC//AfkUUkkIWPJCncdsd7jeXMOENyhdSo1YOFElDrSAX3ZjibeHioHvx6055Ou8XzrLlhT/hQI2RrcVJpNwu/4xvxHWJPuPBcObXHLy47ami7MaV9rOIInppWCsnx33TqjSocCCscnGTclFAbbJRomvczCo9cLB63jZ77kReVF7VNX4GmOxyNx3xsHCI79EptzqpJDIEoGcL87I4u58lszEZ6JpTW1AEkb927VmmeDPpmo9st/+1G4I4IvudLtdqAQX5Pvepu5m8wXClOE4SWW09HK1qENij6IbGjaPS+HK4Go0dcSeFR4nCH/QQZos66R4cSRE1mrqBGsmsVqwU2Jcm7YSEvZwTWCk6M6Pz";

    private TFObjectDetector tfod;

    RobotHardware robotHardware;
    RobotProfile robotProfile;
    HardwareMap hardwareMap;

    CVPipeline pipeline = new CVPipeline();
    int imgcnt = 0;

    public RobotVision(RobotHardware robotHardware, RobotProfile robotProfile) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.hardwareMap = robotHardware.getHardwareMap();
    }

    public void init() {
        init(true);
    }

    public void init(boolean vuforia){
        Logger.logFile("RobotVision init()");
        //TODO - enable the following
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        initVuforia();
        MASK_LOWER_BOUND_H = robotProfile.cvParam.maskLowerH;
        MASK_LOWER_BOUND_S = robotProfile.cvParam.maskLowerS;
        MASK_LOWER_BOUND_V = robotProfile.cvParam.maskLowerV;
        MASK_UPPER_BOUND_H = robotProfile.cvParam.maskUpperH;
        MASK_UPPER_BOUND_S = robotProfile.cvParam.maskUpperS;
        MASK_UPPER_BOUND_V = robotProfile.cvParam.maskUpperV;
        CROP_TOP_PERCENT = robotProfile.cvParam.cropTop;
        MIN_AREA = robotProfile.cvParam.minArea;
        saveImage = true;
     }

    private void initVuforia() {
        Logger.logFile("initVuforia " + this);
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Set RGB format for OpenCV to use
        // Set queue capacity to 1 so OpenCV can grab the latest frame
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        // No need for tracking target for now
        // trackingTargetSetup(parameters)
        Logger.logFile("Vuforia Initialized");
    }

    public void stopVuforia() {
        vuforia.getCamera().close();
    }


    void trackingTargetSetup(VuforiaLocalizer.Parameters parameters) {
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);



        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = robotProfile.hardwareSpec.cameraForwardDisplacement * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = robotProfile.hardwareSpec.cameraVerticalDisplacement * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = robotProfile.hardwareSpec.cameraLeftDisplacement * mmPerInch;     // eg: Camera is ON the robot's center line
        //phoneZRotate = robotProfile.hardwareSpec.cameraHeadingOffset;
        Logger.logFile("Camera Heading Offset:" + phoneZRotate);
        Logger.logFile("Camera Forward:" + CAMERA_FORWARD_DISPLACEMENT);
        Logger.logFile("Camera Left:" + CAMERA_LEFT_DISPLACEMENT);
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            //((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, robotFromCamera);
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    public boolean isTargetVisible() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                return true;
            }
        }
        return false;
    }
    public Pose2d getNavigationLocalization() {
        double x, y, heading;
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            x = translation.get(0) / mmPerInch;
            y = translation.get(1) / mmPerInch;

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, AngleUnit.RADIANS);
            heading = rotation.thirdAngle;
            Logger.logFile("Orientation:" + Math.toDegrees(rotation.firstAngle) + ", " +
                    Math.toDegrees(rotation.secondAngle) + ", " +  Math.toDegrees(rotation.thirdAngle));
            return new Pose2d(new Vector2d(x, y), heading-Math.PI/2);
        }
        else {
            return null;
        }
    }

    public void activateNavigationTarget(){
        Logger.logFile("Activate Navigation Targest " + targets.size());
        targets.activate();
    }

    public void deactivateNavigationTarget(){
        Logger.logFile("Deactivate navigation targets ");
        targets.deactivate();
    }
/*
    public void activateRecognition(){
        Logger.logFile("activateRecognition " + this + " tfod " + tfod);
        tfod.activate();
        tfod.setZoom(2, 1.33);
    }

    public void deactivateRecognition(){
        Logger.logFile("deactivate recognition " + this + " tfod " + tfod);
        tfod.deactivate();
    }

    public List<Recognition> getRingRecognition(){
        return tfod.getRecognitions();
    }
*/
    public AutonomousGoal getAutonomousRecognition(boolean isRed){
        return getAutonomousRecognition(true, isRed);
    }

    public AutonomousGoal getAutonomousRecognition(boolean keepImg, boolean isRed){
        this.saveImage = keepImg;

        handleRingImage();

        ArrayList<Rect> rects = pipeline.getRingRecList();

        if(isRed && rects.size() == 0) {
            return AutonomousGoal.LEFT;
        }else if(!isRed && rects.size() == 0){
            return  AutonomousGoal.RIGHT;
        }
        Rect r = rects.get(0);
        int objCenter = r.x + r.width/2;
        if (isRed) {
            if (objCenter > 400) {
                return AutonomousGoal.RIGHT;
            } else if (objCenter > 220) {
                return AutonomousGoal.MIDDLE;
            } else {
                return AutonomousGoal.LEFT;
            }
        }
        else {
            if (objCenter > 450) {
                return AutonomousGoal.RIGHT;
            } else if (objCenter  > 220 ) {
                return AutonomousGoal.MIDDLE;
            } else {
                return AutonomousGoal.LEFT;
            }
        }
    }

    private Mat getCameraImage() {
        try {
            Image rgb = null;
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
            long numImages = frame.getNumImages();

            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                    break;
                }
            }

            /*rgb is now the Image object that we've used in the video*/
            if (rgb!=null) {
                Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(rgb.getPixels());

                //put the image into a MAT for OpenCV
                Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
                Utils.bitmapToMat(bm, tmp);
                frame.close();
                return tmp;
            }
            //close the frame, prevents memory leaks and crashing
            frame.close();
        }
        catch (InterruptedException ex) {
        }
        return null;
    }

    private void handleRingImage() {
        Mat img = getCameraImage();
        pipeline.processFrame(img);
    }

    private Mat getHsvMask(Mat hsvMat, Scalar lowerBound, Scalar upperBound) {
        Mat mask = new Mat();
        if (lowerBound.val[0]<upperBound.val[0]) {
            Core.inRange(hsvMat, lowerBound, upperBound, mask);
            return mask;
        }
        Mat m1 = new Mat();
        Mat m2 = new Mat();
        Core.inRange(hsvMat, new Scalar(0, lowerBound.val[1], lowerBound.val[2]), new Scalar(lowerBound.val[0], upperBound.val[1], upperBound.val[2]), m1);
        Core.inRange(hsvMat, new Scalar(upperBound.val[0], lowerBound.val[1], lowerBound.val[2]), new Scalar(255, upperBound.val[1], upperBound.val[2]), m2);
        Core.add(m1, m2, mask);
        return mask;
    }

    private void saveImage(Mat mat, String filename) {
        Mat mbgr = new Mat();
        Imgproc.cvtColor(mat, mbgr, Imgproc.COLOR_RGB2BGR, 3);
        Imgcodecs.imwrite("/sdcard/FIRST/" + filename + ".jpg", mbgr);
        mbgr.release();
    }

    public void saveNextImage() {
        saveImage = true;
    }

    public static int MASK_LOWER_BOUND_H = 20;
    public static int MASK_LOWER_BOUND_S = 150;
    public static int MASK_LOWER_BOUND_V = 100;
    public static int MASK_UPPER_BOUND_H = 30;
    public static int MASK_UPPER_BOUND_S = 255;
    public static int MASK_UPPER_BOUND_V = 255;
    public static int MIN_AREA = 5;
    public static int DIM_MULTIPLIER = 4;   // because we pyrDown twice, each with default factor of 2
    public static int CROP_TOP_PERCENT = 20;
    public static int CROP_LEFT_PERCENT = 0;
    public static int CROP_RIGHT_PERCENT = 0;
    public static int CROP_BOTTOM_PERCENT = 0;
    static Scalar DRAW_COLOR_RED = new Scalar(255, 0, 0);
    static Scalar DRAW_COLOR_GREEN = new Scalar(0, 255, 0);
    static boolean saveImage = false;

    static class RectComparator implements Comparator<Rect> {
        @Override
        public int compare(Rect rect, Rect t1) {
            return t1.width*t1.height - rect.width*rect.height;
        }
    }

    public static class CVPipeline extends OpenCvPipeline {
        int count;

        // Cache
        Mat procMat;
        Mat pyrDownMat = new Mat();
        Mat hsvMat = new Mat();
        Mat maskMat = new Mat();
        Mat dilatedMask = new Mat();
        Mat hierarchey = new Mat();
        ArrayList<Rect> ringRecList = new ArrayList<Rect>();

        @Override
        public Mat processFrame(Mat input) {
            int offsetX;
            int offsetY;
            if (CROP_TOP_PERCENT!=0 || CROP_BOTTOM_PERCENT!=0 || CROP_RIGHT_PERCENT!=0 || CROP_LEFT_PERCENT!=0) {
                offsetX = input.width()*CROP_LEFT_PERCENT/100;
                offsetY = input.height()*CROP_TOP_PERCENT/100;
                procMat = input.submat(new Rect(offsetX, offsetY,
                        input.width()*(100-CROP_LEFT_PERCENT-CROP_RIGHT_PERCENT)/100,
                        input.height()*(100-CROP_TOP_PERCENT-CROP_BOTTOM_PERCENT)/100));
            }
            else {
                offsetX = 0;
                offsetY = 0;
                procMat = input;
            }
            Log.i("OpenCV", "Offset:" + offsetX + "," + offsetY);
            Log.i("OpenCV", "CropImg:" + procMat.width() + "," + procMat.height());
            Imgproc.pyrDown(procMat, pyrDownMat);
            Imgproc.pyrDown(pyrDownMat, pyrDownMat);

            Imgproc.cvtColor(pyrDownMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
            Scalar lowerBound = new Scalar(MASK_LOWER_BOUND_H, MASK_LOWER_BOUND_S, MASK_LOWER_BOUND_V);
            Scalar upperBound = new Scalar(MASK_UPPER_BOUND_H, MASK_UPPER_BOUND_S, MASK_UPPER_BOUND_V);
            Core.inRange(hsvMat, lowerBound, upperBound, maskMat);
            Imgproc.dilate(maskMat, dilatedMask, new Mat());

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Imgproc.findContours(dilatedMask, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Iterator<MatOfPoint> each = contours.iterator();
            ringRecList.clear();
            int ndx = 0;
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                if (area > MIN_AREA) {
                    Rect rec = Imgproc.boundingRect(wrapper);
                    rec.x = rec.x * DIM_MULTIPLIER + offsetX;
                    rec.y = rec.y * DIM_MULTIPLIER + offsetY;
                    rec.width *= DIM_MULTIPLIER;
                    rec.height *= DIM_MULTIPLIER;
                    ringRecList.add(rec);
                    //Rect drawRec = new Rect(rec.x*DIM_MULTIPLIER, rec.y*DIM_MULTIPLIER, rec.width*DIM_MULTIPLIER, rec.height*DIM_MULTIPLIER);
                    if (saveImage) {    // update drawing only when saving the picture
                        Imgproc.rectangle(input, rec, DRAW_COLOR_RED, 2);
                    }
                    Log.i("Area", "Ndx:" + ndx + " Area:" + area + " Rec:" + rec.x + "," + rec.y + "," + rec.width + "," + rec.height);
                    count++;
                }
                ndx++;
            }

            if(saveImage){
                //need to save pic to file
                String timestamp = new SimpleDateFormat("MMdd-HHmmss", Locale.US).format(new Date());
                Mat mbgr = new Mat();
                Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
                Imgcodecs.imwrite("/sdcard/FIRST/GOAL" + timestamp + ".jpg", mbgr);
                mbgr.release();
            }

            for (Rect r : ringRecList) {
//                Logger.logFile("Vision Rec:" + r);
            }
//            Logger.logFile("Sorting...");
            Collections.sort(ringRecList, new RectComparator());
            for (Rect r : ringRecList) {
//                Logger.logFile("Vision Rec:" + r);
            }
            return input;
        }

        public ArrayList<Rect> getRingRecList() {
            return ringRecList;
        }
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    int nHubPic = 0;
    long lastTime = 0;
    double lastTps = 0;
    public void initRearCamera(boolean isRed) {
//        lastHubResult = null;
        rearCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam2"));
        rearCamera.setMillisecondsPermissionTimeout(2500);
        lastTime = System.currentTimeMillis();
        rearCamera.openCameraDeviceAsync(
            new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    RobotLog.i("RearCamera opened");
                    Logger.logFile("RearCamera opened");
//                    rearCamera.setPipeline(new HubAutodriveVision2(isRed));
                }

                @Override
                public void onError(int errorCode) {
                    Logger.logFile("Failed to open RearCamera: " + errorCode);
                    RobotLog.e("Failed to open RearCamera: " + errorCode);
                }
            }
        );
    }
//
//    HubVisionMathModel.Result lastHubResult;
//
//    HubVisionMathModel.Result getLastHubResult() {
//        return lastHubResult;
//    }
//
//    class HubAutodriveVision2 extends OpenCvPipeline {
//        Mat hsvMat = new Mat();
//        Mat maskMat = new Mat();
//        Mat dilatedMat = new Mat();
//        Mat hierarchey = new Mat();
//        HubVisionMathModel model;
//        boolean isRed;
//
//        public HubAutodriveVision2(boolean isRed) {
//            model = new HubVisionMathModel(robotProfile);
//            this.isRed = isRed;
//        }
//
//        protected void finalized() {
//            hsvMat.release();
//            maskMat.release();
//            dilatedMat.release();
//            hierarchey.release();
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            model = new HubVisionMathModel(robotProfile);
//            // 0. Crop the middle part
//            Mat workMat = input.submat(new Rect(0, robotProfile.hvParam.cropTop, input.width(), robotProfile.hvParam.cropBottom));
//            // 1. Convert input to HSV
//            Imgproc.cvtColor(workMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
//            Scalar maskLow = (isRed)?robotProfile.hvParam.maskLowRed:robotProfile.hvParam.maskLowBlue;
//            Scalar maskHigh = (isRed)?robotProfile.hvParam.maskHighRed:robotProfile.hvParam.maskHighBlue;
//            // 2. Create MASK
//            if (maskLow.val[0] > maskHigh.val[0]) {
//                // RED situation
//                Mat maskMat1 = new Mat();
//                Mat maskMat2 = new Mat();
//                Core.inRange(hsvMat, maskLow,
//                        new Scalar(255, maskHigh.val[1], maskHigh.val[2]), maskMat1);
//                Core.inRange(hsvMat, new Scalar(0, maskLow.val[1], maskLow.val[2]),
//                        maskHigh, maskMat2);
//                Core.add(maskMat1, maskMat2, maskMat);
//                maskMat1.release();
//                maskMat2.release();
//            } else {
//                // Non RED situation
//                Core.inRange(hsvMat, maskLow, maskHigh, maskMat);
//            }
//            Imgproc.dilate(maskMat, dilatedMat, new Mat());
//            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
//            Imgproc.findContours(dilatedMat, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//            Iterator<MatOfPoint> each = contours.iterator();
//            int ndx = 0;
//            while (each.hasNext()) {
//                MatOfPoint wrapper = each.next();
//                double area = Imgproc.contourArea(wrapper);
//                if (area > robotProfile.hvParam.minArea) {
//                    Rect rec = Imgproc.boundingRect(wrapper);
//                    if (rec.height> robotProfile.hvParam.minHeight) {
//                        model.addRect(rec);
//                    }
//                    if (saveImage) {    // update drawing only when saving the picture
//                        Imgproc.rectangle(input, new Rect(rec.x, rec.y+robotProfile.hvParam.cropTop, rec.width, rec.height),
//                                DRAW_COLOR_RED, 2);
//                    }
//                    Log.i("Vision", " x:" + rec.x + " w:" + rec.width + " h:" + rec.height);
//                }
//                else {
//                    Log.i("Vision", "Ndx:" + ndx + " Area:" + area);
//                }
//                ndx++;
//            }
//            lastHubResult = model.getResult();
//            nHubPic++;
//            if (nHubPic%10==0) {
//                long t = System.currentTimeMillis();
//                lastTps = 10.0 / (t - lastTime);
//                lastTime = t;
//            }
//            if (saveImage) {
//                SavePicturePipeline sp = new SavePicturePipeline();
//                sp.processFrame(input);
//                savePicture(maskMat);
//                saveImage = false;
//            }
//            return input;
//        }
//        public Mat savePicture(Mat input) {
//            String timestamp = new SimpleDateFormat("MMdd-HHmmssSSS", Locale.US).format(new Date());
//            Mat mbgr = new Mat();
//            Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
//            Imgcodecs.imwrite("/sdcard/FIRST/MASK-" + timestamp + ".jpg", mbgr);
//            mbgr.release();
//            return input;
//        }
//    }
//
//    class HubAutodriveVision extends OpenCvPipeline {
//        Mat hsvMat = new Mat();
//        Mat maskMat = new Mat();
//        Mat dilatedMat = new Mat();
//        Mat dilatedMat2 = new Mat();
//        Mat blurredMat = new Mat();
//        Mat lines = new Mat();
//        Mat maskMat1 = new Mat();
//        Mat maskMat2 = new Mat();
//        Mat edgesMat = new Mat();
//
//        public HubAutodriveVision() {
//        }
//
//        protected void finalized() {
//            hsvMat.release();
//            maskMat.release();
//            dilatedMat.release();
//            dilatedMat2.release();
//            edgesMat.release();
//            lines.release();
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//
//            HubVisionMathModel model = new HubVisionMathModel(robotProfile);
//
//            // 0. Crop the middle part
//            Mat workMat = input.submat(new Rect(0, robotProfile.hvParam.cropTop, input.width(), robotProfile.hvParam.cropBottom));
//            // 1. Convert input to HSV
//            Imgproc.cvtColor(workMat, hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
//
//            // 2. Create MASK
//            if (robotProfile.hvParam.maskLowRed.val[0]>robotProfile.hvParam.maskHighRed.val[0]) {
//                // RED situation
//                Core.inRange(hsvMat, robotProfile.hvParam.maskLowRed,
//                        new Scalar(255, robotProfile.hvParam.maskHighRed.val[1],robotProfile.hvParam.maskHighRed.val[2]), maskMat1);
//                Core.inRange(hsvMat, new Scalar(0, robotProfile.hvParam.maskLowRed.val[1], robotProfile.hvParam.maskLowRed.val[2]),
//                        robotProfile.hvParam.maskHighRed, maskMat2);
//                Core.add(maskMat1, maskMat2, maskMat);
//                maskMat1.release();
//                maskMat2.release();
//            }
//            else {
//                // Non RED situation
//                Core.inRange(hsvMat, robotProfile.hvParam.maskLowRed, robotProfile.hvParam.maskHighRed, maskMat);
//            }
//            // 3. Dilate to fill the gaps
//            Imgproc.dilate(maskMat, dilatedMat, new Mat());
//            // 4. Blur the image
//            Imgproc.GaussianBlur(maskMat, blurredMat, new org.opencv.core.Size(3, 3), 3);
//            // 5. Canny edge detection
//            Imgproc.Canny(blurredMat, edgesMat, 100, 70 * 3, 7, false);
//            // 6. Dilate again before finding lines
//            Imgproc.dilate(edgesMat, dilatedMat2, new Mat(), new Point(-1, -1), 1); // 1
//            // 7. Finding the lines
//            int lineGap = 15;
//            Imgproc.HoughLinesP(dilatedMat2, lines, 3, Math.PI / 1000, 2, 45, lineGap);
//            System.out.println("Got lines: " + lines.rows());
//            for (int i = 0; i < lines.rows(); i++) {
//                double[] val = lines.get(i, 0);
////                Point tmpStartP = new Point(val[0], val[1] + robotProfile.hvParam.CROP_TOP);
////                Point tmpEndP = new Point(val[2], val[3] + robotProfile.hvParam.CROP_TOP);
//                model.addLine((int)val[0], (int)val[1], (int)val[2], (int)val[3]);
////                if (Math.abs(val[0]-val[2])<10) {
////                    Imgproc.line(clone, tmpStartP, tmpEndP, DRAW_COLOR2, 1);
////                    System.out.println("   " + val[0] + "," + val[1] + " -> " + val[2] + "," + val[3]);
////                }
////                else {
////                    Imgproc.line(clone, tmpStartP, tmpEndP, DRAW_COLOR3, 1);
////                    System.out.println("xxx" + val[0] + "," + val[1] + " -> " + val[2] + "," + val[3]);
////                }
//            }
//            if (model.getResult().result== HubVisionMathModel.RecognitionResult.BOTH) {
//                lastHubResult = model.getResult();
//            }
//            nHubPic++;
//            if (nHubPic%10==0) {
//                long t = System.currentTimeMillis();
//                lastTps = 10.0 / (t - lastTime);
//                lastTime = t;
//                //savePicture(input);
//            }
//            workMat.release();
//            return input;
//        }
//
//        public Mat savePicture(Mat input) {
//            String timestamp = new SimpleDateFormat("MMdd-HHmmssSSS", Locale.US).format(new Date());
//            Mat mbgr = new Mat();
//            Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
//            Imgcodecs.imwrite("/sdcard/FIRST/REAR-" + timestamp + ".jpg", mbgr);
//            mbgr.release();
//            return input;
//        }
//    }
//    public double getHubVicTps() {
//        return lastTps;
//    }

    class SavePicturePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            if (saveImage) {
                String timestamp = new SimpleDateFormat("MMdd-HHmmssSSS", Locale.US).format(new Date());
                Mat mbgr = new Mat();
                Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
                Imgcodecs.imwrite("/sdcard/FIRST/REAR-" + timestamp + ".jpg", mbgr);
                mbgr.release();
                saveImage = false;
            }
            return input;
        }
    }

    public void startRearCamera() {
        rearCamera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public boolean isRearCameraOpened() {
        return rearCamera!=null;
    }

    public void stopRearCamera() {
        rearCamera.stopStreaming();
        rearCamera.closeCameraDevice();
    }
}

