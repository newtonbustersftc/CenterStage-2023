package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.opencsv.CSVWriter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Logger;
import org.firstinspires.ftc.teamcode.RobotFactory;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotProfile;
import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import android.util.Log;

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
    public static double DISTANCE = 35; // in

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private NBMecanumDrive drive;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    TrajectoryVelocityConstraint velConstraint = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    TrajectoryAccelerationConstraint accelConstraint = getAccelerationConstraint(MAX_ACCEL);
    private Mode mode;

    //testing for tracking wheels' encoder counts with csv output file:
    String[] header = { "Time", "FLMotor", "FRMotor","RLMotor", "CurrentX","CurrentY", "TargetVelocity",
                        "CurrentVelocity", "ErrorVelocity"};
    FileOutputStream fos;
    OutputStreamWriter osw;
    CSVWriter inputFile;
    ArrayList<Double> timeArray = new ArrayList();
    ArrayList<Integer> FLEncoderArray = new ArrayList();
    ArrayList<Integer> FREncoderArray = new ArrayList();
    ArrayList<Integer> RLEncoderArray = new ArrayList();
    ArrayList<Double> CurrentXArray = new ArrayList<>();
    ArrayList<Double> CurrentYArray = new ArrayList<>();
    ArrayList<Double> TargetVelocityArray = new ArrayList<>();
    ArrayList<Double> CurrentVelocityArray = new ArrayList<>();
    ArrayList<Double> ErrorVelocityArray = new ArrayList<>();
    //end of csv output file declaration

    private static MotionProfile generateProfile(boolean movingForward) {
        Logger.logFile("movingForward : "+movingForward);

        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }

    @Override
    public void runOpMode() {
        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        NBMecanumDrive drive;
       RobotProfile robotProfile = null;
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        Logger.init();
        RobotHardware robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
        robotHardware.resetDriveAndEncoders();
        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));

        mode = Mode.TUNING_MODE;

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        //testing tracking wheels with csv file as output
        try {
            fos = new FileOutputStream("/sdcard/FIRST/ManualFFTuner2022.csv");
            osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
            inputFile = new CSVWriter(osw);
            // adding header to csv
            inputFile.writeNext(header);
            Logger.logFile("inputFile:"+inputFile.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
        //end of tracking wheels csv file

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();
        Logger.logFile("test 8***startTime... : "+profileStart);


        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE, velConstraint, accelConstraint)
                .build();

//        Trajectory trajectoryBackward = new TrajectoryBuilder(trajectoryForward.end(),Math.toRadians(180)/*trajectoryForward.end().getHeading()*/,
//                velConstraint, accelConstraint)
//                .back(DISTANCE)
//                .build();
        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end(),true)/*trajectoryForward.end().getHeading()*/
                .back(DISTANCE)
                .build();

        drive.followTrajectoryAsync(trajectoryForward); //start with forward run

        while (opModeIsActive() && !isStopRequested()) {
            robotHardware.clearBulkCache();
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;
                    Logger.logFile("profileTime: "+profileTime);

//                    if (profileTime > trajectory.duration()) {
                    boolean switchDirection = (Math.abs(drive.getPoseEstimate().getX()) > DISTANCE);
                    if(switchDirection){
                        movingForwards = !movingForwards;
                        Logger.logFile("movinForward:"+movingForwards);
                        profileStart = clock.seconds();

                        robotHardware.resetDriveAndEncoders();
                        robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));

                        if(!movingForwards) {
                            Logger.logFile("movingForward is false, trajectoryBackward");
                            drive.followTrajectoryAsync(trajectoryBackward);
                        }else{
                            Logger.logFile("movingForward is true, trajectoryForward");
                            drive.followTrajectoryAsync(trajectoryForward);
                        }

                        Logger.logFile("trajectory start X:"+ (movingForwards ? trajectoryForward.start().getX() : trajectoryBackward.start().getX()));
                        Logger.logFile("trajectory end X:" + (movingForwards ? trajectoryForward.end().getX() : trajectoryBackward.end().getX()));
                        Logger.logFile("trajectory duration:"+ trajectoryForward.duration());
                        Logger.logFile("new trajectory s" +
                                "tarts at localizer:" + robotHardware.getLocalizer().getPoseEstimate());
                    }

//                    MotionState motionState = activeProfile.get(profileTime);
//                    double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);

                    MotionState motionState=null;
                    if(movingForwards)
                        motionState = trajectoryForward.getProfile().get(profileTime);
                    else
                        motionState = trajectoryBackward.getProfile().get(profileTime);
                    double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(),kV, kA,kStatic);

                    Logger.logFile("target power: " + targetPower);
                    Logger.flushToFile();
                    if(movingForwards)
                        drive.setDrivePower(new Pose2d(targetPower, 0, 0));
                    else
                        drive.setDrivePower(new Pose2d(-targetPower, 0, 0));

                    drive.updatePoseEstimate();
                    Logger.logFile("X:" +drive.getPoseEstimate().getX());
                    Logger.logFile("Y:" + drive.getPoseEstimate().getY());
                    Logger.logFile("heading:"+ drive.getPoseEstimate().getHeading());
                    Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
                    double currentVelo = poseVelo.getX();

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.getV());
                    telemetry.addData("measuredVelocity", -currentVelo);
                    telemetry.addData("error", motionState.getV() + currentVelo);

                    //testing tracking wheels encoder count and output to csv file:
                    timeArray.add((double)(clock.seconds() - profileStart));
                    FLEncoderArray.add(robotHardware.getFLMotorEncoderCnt());
                    FREncoderArray.add(robotHardware.getFRMotorEncoderCnt());
                    RLEncoderArray.add(robotHardware.getRlMotorEncoderCnt());
                    CurrentXArray.add(drive.getPoseEstimate().getX());
                    CurrentYArray.add(drive.getPoseEstimate().getY());
                    TargetVelocityArray.add(motionState.getV());
                    CurrentVelocityArray.add(-currentVelo);
                    ErrorVelocityArray.add(motionState.getV() - currentVelo);
                    //end of tracking wheels csv file

                    break;
                case DRIVER_MODE:
                    //original version of generate motion profile
//                    MotionProfile activeProfile = generateProfile(true);
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
        //tracking wheels encoder count csv file
        for(int idx=0; idx<timeArray.size(); idx++){
            String[] data = {timeArray.get(idx)+"", FLEncoderArray.get(idx)+"",
                    FREncoderArray.get(idx)+"",
                    RLEncoderArray.get(idx)+"",
                    CurrentXArray.get(idx)+ "",
                    CurrentYArray.get(idx)+ "",
                    TargetVelocityArray.get(idx)+"",
                    CurrentVelocityArray.get(idx)+"",
                    ErrorVelocityArray.get(idx)+""};

            inputFile.writeNext(data);
        }
        try {
            inputFile.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
        Logger.logFile("total count: "+timeArray.size());
        //end of tracking csv file

        Logger.flushToFile();
        telemetry.update();
    }
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}