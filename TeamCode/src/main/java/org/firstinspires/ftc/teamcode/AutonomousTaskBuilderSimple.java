package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;
import java.util.Arrays;

//Less code is always better than more code, less debugging and testing needed
public class AutonomousTaskBuilderSimple {
    RobotProfile robotProfile;    RobotHardware robotHardware;
    ArrayList<RobotControl> taskList = new ArrayList<>();
    NBMecanumDrive drive;
    String delayString, startPosMode, passThrough, parking;
    Pose2d startingPose, aprilTagPt;
    String teamPropPos; //default in case
    PixelBoardVision aprilTagRecognition;
    boolean isRed, isFar;

    public AutonomousTaskBuilderSimple(RobotHardware robotHardware, RobotProfile robotProfile,
                                       PixelBoardVision aprilTagRecognition, String teamPropPos, Pose2d startingPose) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        drive = (NBMecanumDrive) robotHardware.getMecanumDrive();
        this.teamPropPos = teamPropPos;
        this.startingPose = startingPose;
        this.aprilTagRecognition = aprilTagRecognition;
    }

    public ArrayList<RobotControl> buildTaskList() {
        try {
            SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
            delayString = "0";
            startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);;
            delayString = "0";
            Logger.logFile(AutonomousOptions.START_POS_MODES_PREF + " - " + startPosMode);
            Logger.logFile(AutonomousOptions.START_DELAY_PREF + " - " + delayString);
            if(startPosMode.startsWith("RED")){
                isRed = true;
            }
            isFar = (startPosMode.equals("BLUE_RIGHT") || startPosMode.equals("RED_LEFT"));
            parking = (isFar)?"MIDDLE":"CORNER";
        } catch (Exception e) {
            RobotLog.e("SharedPref exception " + e);
            this.delayString = "0";
        }
        Logger.logFile("Done with init in autonomous - team prop " + teamPropPos);

        RobotProfile.AutonParam param = robotProfile.autonParam;
        TrajectoryVelocityConstraint velFast = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), robotProfile.hardwareSpec.trackWidth);
        TrajectoryAccelerationConstraint accelFast = getAccelerationConstraint(param.fastAcceleration);
        TrajectoryVelocityConstraint velConstraint = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), 3);
        TrajectoryAccelerationConstraint accelConstraint = getAccelerationConstraint(param.normAcceleration);
        // THIS IS HERE FOR NOW
        Pose2d parkingPose1 = null, parkingPose2 = null;
        parkingPose1 = robotProfile.getProfilePose("PARKING_" + (isRed?"RED":"BLUE") + "_" + (isFar?"MIDDLE":"CORNER"));
        parkingPose2 = new Pose2d(parkingPose1.getX()+5, parkingPose1.getY(), parkingPose1.getHeading());

        if (!delayString.equals("0")) {
            taskList.add(new RobotSleep(Integer.parseInt(delayString) * 1000));
        }
        Pose2d propPose = robotProfile.getProfilePose("TEAM_PROP_POS_" + teamPropPos + "_" +startPosMode);
        Pose2d dropPose = robotProfile.getProfilePose("DROPBOARD_APRILTAG_" + (isRed?"RED":"BLUE") + "_" + teamPropPos);
        Pose2d parkingPose = robotProfile.getProfilePose("PARKING_" + startPosMode);

        if (!isFar) {
            Pose2d outPose = getProfilePose("HZ_OUT" );
            Pose2d rot1Pose = getProfilePose("HZ_ROT1");
            Pose2d spkPose = getProfilePose("HZ_SPIKE");
            Pose2d aftSpkPose = getProfilePose("HZ_AFT_SPIKE");
            Pose2d pixelPose = getProfilePose("HZ_PIXEL");
            TrajectorySequence spkTrj = drive.trajectorySequenceBuilder(startingPose)
                    .lineToConstantHeading(outPose.vec())
                    .turn(rot1Pose.getHeading() - outPose.getHeading())
                    .lineTo(spkPose.vec())
                    .build();
            taskList.add(new SplineMoveTask(drive, spkTrj));
            // Drop spike by lifting the intake
            taskList.add(new RobotSleep(1000, "DROP SPIKE"));
            // Move to pick up pixel
            TrajectorySequenceBuilder aftSpikeTrjBldr = drive.trajectorySequenceBuilder(spkPose)
                    .lineTo(aftSpkPose.vec())
                    .lineTo(pixelPose.vec());
            taskList.add(new SplineMoveTask(drive, aftSpikeTrjBldr.build()));
            taskList.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.CLOSE));
            taskList.add(new PixelUpTask(robotHardware, false, robotProfile.hardwareSpec.liftOutAuto));
            taskList.add(new DropPixelTask(robotHardware));
        }
        else {
            Pose2d outPose = getProfilePose("HZ_OUT" );
            Pose2d rot1Pose = getProfilePose("HZ_ROT1");
            Pose2d spkPose = getProfilePose("HZ_SPIKE");
            Pose2d aftSpkPose = getProfilePose("HZ_AFT_SPIKE");
            Pose2d rot2Pose = getProfilePose("HZ_ROT2");
            Pose2d prePickPose = getProfilePose("HZ_PRE_PICK");
            Pose2d pickPose = getProfilePose("HZ_PICK");
            Pose2d aftPickPose = getProfilePose("HZ_AFT_PICK");
            Pose2d preAprilPose = getProfilePose("HZ_PRE_APRIL");
            Pose2d aprilPose = getProfilePose("HZ_APRIL");
            // Move out to the drop spike mark
            TrajectorySequence spkTrj = drive.trajectorySequenceBuilder(startingPose)
                    .lineToConstantHeading(outPose.vec())
                    .turn(rot1Pose.getHeading() - outPose.getHeading())
                    .lineTo(spkPose.vec())
                    .build();
            taskList.add(new SplineMoveTask(drive, spkTrj));
            // Drop spike by lifting the intake
            taskList.add(new IntakePositionTask(robotHardware, true));
            // Move to pick up pixel
            TrajectorySequenceBuilder aftSpikeTrjBldr = drive.trajectorySequenceBuilder(spkPose)
                    .lineTo(aftSpkPose.vec());
            if (aftSpkPose.getHeading()!=rot2Pose.getHeading()) {
                aftSpikeTrjBldr.turn(rot2Pose.getHeading() - aftSpkPose.getHeading());
            }
            aftSpikeTrjBldr.lineTo(prePickPose.vec());
            aftSpikeTrjBldr.lineTo(pickPose.vec());
            taskList.add(new SplineMoveTask(drive, aftSpikeTrjBldr.build()));
            // Pick up pixel
            taskList.add(new IntakeActionTask(robotHardware, RobotHardware.IntakeMode.SLOW));
            taskList.add(new RobotSleep(500, "Low Power drag"));
            TrajectorySequence pickMoveBackTrj = drive.trajectorySequenceBuilder(pickPose)
                    .setAccelConstraint(getAccelerationConstraint(5))
                    .setVelConstraint(getVelocityConstraint(5, Math.toRadians(5), 14.0))
                    .lineTo(aftPickPose.vec())
                    .build();
            taskList.add(new SplineMoveTask(drive, pickMoveBackTrj));
            taskList.add(new IntakePositionTask(robotHardware, false));
            taskList.add(new SmartIntakeActionTask(robotHardware, 3000));
            taskList.add(new IntakePositionTask(robotHardware, true));
            // Move to read AprilTag
            TrajectorySequence toAprilTrj = drive.trajectorySequenceBuilder(aftPickPose)
                    .lineTo(preAprilPose.vec())
                    .lineTo(aprilPose.vec())
                    .build();
            taskList.add(new SplineMoveTask(drive, toAprilTrj));
            taskList.add(new IntakePositionTask(robotHardware, false));
            taskList.add(new RobotSleep(300)); // let april tag does recognition
            taskList.add(new AprilTagDetectionTask(robotHardware, this.aprilTagRecognition,
                    robotProfile, teamPropPos, drive, isRed ));
            goToDropBoard();
            taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, parkingPose1, true));
            taskList.add(new SplineMoveTask(robotHardware.mecanumDrive, parkingPose2, true));
            return taskList;
        }
        return taskList;
    }

    void goToDropBoard(){
        if (teamPropPos.equals("RIGHT")) {
            taskList.add(new PixelUpTask(robotHardware, true, robotProfile.hardwareSpec.liftOutAuto));
        }
        else{
            taskList.add(new PixelUpTask(robotHardware, false, robotProfile.hardwareSpec.liftOutAuto));
        }
        taskList.add(new DropPixelTask(robotHardware));
    }

    // Go for team prop specific pose first, if not found, go for the generic
    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name + "_" + teamPropPos + "_" + startPosMode);
        if (ap==null) {
            ap = robotProfile.poses.get(name + "_" + startPosMode);
            if (ap==null) {
                Logger.logFile("Pose2d entry " + robotProfile.poses.get(name + "_" + teamPropPos + "_" + startPosMode) +
                        " OR " + robotProfile.poses.get(name + "_" + teamPropPos + "_" + startPosMode) + " not available");
            }
        }
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
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