package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode1.TurretMotionProfileGenerator;

import java.io.File;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Objects;

/*
 * This routine is designed to tune the open-loop feedforward coefficients. Although it may seem unnecessary,
 * tuning these coefficients is just as important as the positional parameters. Like the other
 * manual tuning routines, this op mode relies heavily upon the dashboard. To access the dashboard,
 * connect your computer to the RC's WiFi network. In your browser, navigate to
 * https://192.168.49.1:8080/dash if you're using the RC phone or https://192.168.43.1:8080/dash if
 * you are using the Control Hub. Once you've successfully connected, start the program, and your
 * robot will begin moving forward and backward according to a motion profile. Your job is to graph
 * the velocity errors over time and adjust the feedforward coefficients. Once you've found a
 * satisfactory set of gains, add them to the appropriate fields in the DriveConstants.java file.
 *
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */
@Config
@Autonomous(group = "drive")
public class ManualFeedforwardTuner extends LinearOpMode {

    public static int TOTAL_REC_CNT = 50000;
    public static double DISTANCE = 60; // in

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private SampleMecanumDrive drive;
    RobotHardware robotHardware;
    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Mode mode;

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
//        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
        return new TurretMotionProfileGenerator().generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);

    }

    @Override
    public void runOpMode() {
        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }
        NBMecanumDrive drive;
        RobotProfile robotProfile = null;
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        Logger.init();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);

        robotHardware.resetDriveAndEncoders();
        robotHardware.resetImu();
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();

        mode = Mode.TUNING_MODE;

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();
        Logger.logFile("profile has " + activeProfile.getSegments().size() + " segments");
        for(int i=0; i<activeProfile.getSegments().size(); i++){
            MotionSegment segment = activeProfile.getSegments().get(i);
            Logger.logFile("start X="+segment.getStart().getX()+", V="+segment.getStart().getV());
            Logger.logFile("segment dt="+segment.getDt());
        }

        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    MotionState motionState = activeProfile.get(profileTime);
                    Logger.logFile("profileTime="+ profileTime);
                    Logger.logFile(motionState.toString());
                    double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);
                    Logger.logFile("power="+targetPower);

                    drive.setDrivePower(new Pose2d(targetPower, 0, 0));
                    drive.updatePoseEstimate();

                    Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
                    double currentVelo = poseVelo.getX();

                    robotHardware.clearBulkCache();
                    recording[loop] = new Record(profileTime,currentVelo,drive.getPoseEstimate().getX() );
                    recording_RR_profile[loop] = new Record_mp(profileTime, targetPower, motionState.getV(), (int)motionState.getX(), motionState.getA());
                    loop++;

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.getV());
                    telemetry.addData("measuredVelocity", currentVelo);
                    telemetry.addData("error", motionState.getV() - currentVelo);
                    break;
                case DRIVER_MODE:
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    break;
            }

            telemetry.update();
        }
        writeResult();
    }

    int loop = 0;

    class Record {
        double ts;  double velocity; double position;
        public Record(double ts, double velocity, double position) {
            this.ts = ts;
            this.velocity = velocity;
            this.position = position;
        }

        public String toString() {
            return "" + ts +  "," + velocity + "," + position;
        }
    }

    class Record_mp {
        double ts; double power; double velocity; int position; double acceleration;
        public Record_mp(double ts, double power, double velocity, int position, double acceleration) {
            this.ts = ts;
            this.power = power;
            this.velocity = velocity;
            this.position = position;
            this.acceleration = acceleration;
        }

        public String toString() {
            return "" + ts + "," + power + "," + velocity + "," + position +","+acceleration;
        }
    }

    Record[] recording = new Record[TOTAL_REC_CNT];
    Record_mp[] recording_RR_profile = new Record_mp[TOTAL_REC_CNT];

    private void writeResult(){
        String timestamp = new SimpleDateFormat("yyyyMMdd-HHmm", Locale.US).format(new Date());
        try {
            PrintWriter pw = new PrintWriter(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/turdat_" + timestamp + ".csv"));
            pw.println("ts,power,velocity,position");
            for(int i=0; i<loop; i++){
                pw.println(recording[i]);
            }
            pw.flush();
            pw.close();

            //test, print the second set of data from motion state
            PrintWriter pw_mp = new PrintWriter(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/turdat_RR" + timestamp + ".csv"));
            pw_mp.println("ts,power,velocity,position");
            for(int i=0; i<loop; i++){
                pw_mp.println(recording_RR_profile[i]);
            }
            pw_mp.flush();
            pw_mp.close();
        }
        catch (Exception ex) {
            ex.printStackTrace();
        }
        try {
            Logger.flushToFile();
        }
        catch (Exception ex) {
        }
    }
}