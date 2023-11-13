package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.opencv.core.Scalar;

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
    public AutonParam autonParam;

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

    public String toString() {
        GsonBuilder builder = new GsonBuilder();
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        return gson.toJson(this);
    }

    public class PIDParam {
        public double p;
        public double i;
        public double d;
        public double f;
    }

    public class HardwareSpec {
        public boolean revHubVertical;
        public boolean useControlHubImu;
        public double ticksPerRev;
        public double trackWidth;
        public double encoderOffset;
        public double wheelRadius;
        public double forwardOffset;
        public double wheelBase;
        public boolean leftEncoderReverse;
        public boolean rightEncoderReverse;
        public boolean horizontalEncoderReverse;
        public int liftMax;
        public int liftOutMin;
        public double liftPowerUp, liftPowerDown;
        public double grabberOpenPos;
        public double grabberClosePos;
        public double droneServoInitPos;
        public double droneServoShootPos;
        public double droneHookClosePos;
        public double droneHookOpenPos;
        public double intakePower;
        public double intakeServo1In, intakeServo1Out, intakeServo1Stop;
        public double intakeServo2In, intakeServo2Out, intakeServo2Stop;
        public double gripperInOutServoOut, gripperInOutServoIn;
        public double gripperRotateServoUp, gripperRotateServoLeft, gripperRotateServoRight;

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
        Scalar redLowerBound;
        Scalar redUpperBound;
        Scalar blueLowerBound;
        Scalar blueUpperBound;
        int cropTopPercent;
        int cropBottomPercent;
        int cropLeftPercent;
        int cropRightPercent;
        int minArea;
    }

    class AutonParam {
        double fastVelocity;
        double fastAngVelo;
        double fastAcceleration;
        double normVelocity;
        double normAngVelo;
        double normAcceleration;
    }

    public void createSampleProfile() {
        hardwareSpec = new HardwareSpec();
        hardwareSpec.revHubVertical = true;
        hardwareSpec.liftMax = 1000;
        hardwareSpec.liftOutMin = 500;

    }
}


