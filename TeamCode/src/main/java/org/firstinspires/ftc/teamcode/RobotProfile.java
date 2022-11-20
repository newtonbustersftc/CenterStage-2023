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
    public PoleParameter poleParameter;

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
        public double grabberInitPos, grabberOpenPos, grabberClosePos;
        public double extensionInitPos, extensionFullOutPos, extensionDriverMin, extensionFullInPos;
        public double turretPower;
        public int turretMaxAhead;
        public int turretMoveMax;
        public int turret360;
        public int liftMax;
        public int liftSafeRotate;
        public double liftPowerUp, liftPowerDown;
        int liftPickPos[];
        int liftDropPos[];
    }

    public class PoleParameter {
        public int samplesEachSide;
        public int[] cropXywh;
        public int[] centerPosition;
        public int closestPoleWidth;
        public int farthestPoleWidth;
        public double farthestArmExtension;
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
        Scalar greenLowerBound;
        Scalar greenUpperBound;
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
        int liftHighDrop;
        int liftStack5;
        int liftStack4;
        int liftUpSafe;
        double armLengthDrop;
        double armLengthPick;
        double armLengthPostPick;
        int turretForwardPos;
        int turretDropPosRight;
        int turretPickPosRight;
        int turretDropPosLeft;
        int turretPickPosLeft;
        double forwardPre1;
        double forward1;
        double forward2;
        double back1;
        double backPick;
    }

    public void createSampleProfile() {
        hardwareSpec = new HardwareSpec();
        hardwareSpec.revHubVertical = true;
        hardwareSpec.liftPickPos = new int[]{10, 12, 36, 10};

    }
}


