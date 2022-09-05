package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.HashMap;

public class RobotProfile {

    PIDParam headingPID;
    PIDParam distancePID;
    public PIDParam rrHeadingPID;
    public PIDParam rrTranslationPID;
    public HardwareSpec hardwareSpec;
    public RoadRunnerParam rrParam;
    public CVParam cvParam;

    HashMap<String, AutoPose> poses;
    Movement movement;
    String fileDateStr;

    public static RobotProfile loadFromFile(File f) throws FileNotFoundException {
        return loadFromFile();
    }

    public static RobotProfile loadFromFile() throws FileNotFoundException {
        SimpleDateFormat sdf = new SimpleDateFormat("MM/DD HH:mm:ss");

        File file1 = new File("/sdcard/FIRST/profileA.json");
        if (!file1.exists()) {
            file1 = new File("/sdcard/FIRST/profileB.json");
        }
        Gson gson = new Gson();
        RobotProfile profile = gson.fromJson(new FileReader(file1), RobotProfile.class);
        profile.fileDateStr = sdf.format(new java.util.Date(file1.lastModified()));
        return profile;
    }

    public Pose2d getProfilePose(String name) {
        AutoPose ap = poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    public void saveToFile(File file) throws FileNotFoundException {
        PrintWriter out = new PrintWriter(file);
        GsonBuilder builder = new GsonBuilder();
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String json = gson.toJson(this);
        out.println(json);
        out.close();
    }

    public class PIDParam {
        public double p;
        public double i;
        public double d;
        public double f;
    }

    public class HardwareSpec {
        public boolean revHubVertical;
        public double ticksPerRev;
        public double trackWidth;
        public double wheelRadius;
        public double forwardOffset;
        public double wheelBase;
        public boolean leftEncoderReverse;
        public boolean rightEncoderReverse;
        public boolean horizontalEncoderReverse;
        public float cameraForwardDisplacement, cameraVerticalDisplacement, cameraLeftDisplacement;
    }

    public class RoadRunnerParam {
        public double kA;
        public double kV;
        public double kStatic;
        public double lateralMultiplier;
        public double maxVel;
        public double maxAcc;
    }

    public class AutoPose {
        double x;
        double y;
        double heading;

        AutoPose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    class Movement {
        double strifeStopDist;
        double forwardStopDist;
        double rotateStopAngle;
    }

    class CVParam {
        int maskUpperH;
        int maskUpperS;
        int maskUpperV;
        int maskLowerH;
        int maskLowerS;
        int maskLowerV;
        int cropTop;
        int cropBottom;
        int minArea;
    }

    public void createSampleProfile() {
        hardwareSpec = new HardwareSpec();
        hardwareSpec.revHubVertical = true;
    }
}


