package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Arrays;

public class AutonomousTaskBuilder {
    RobotProfile robotProfile;
    RobotHardware robotHardware;
    ArrayList<RobotControl> taskList = new ArrayList<RobotControl>();
    NBMecanumDrive drive;
    Pose2d startPos = new Pose2d();
    String delayString;
    String startPosMode;
    String parkingRow;
    SignalRecognition signalRec;

    public AutonomousTaskBuilder(RobotHardware robotHardware, RobotProfile robotProfile, SignalRecognition signalRec) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.signalRec = signalRec;
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
    }

    public ArrayList<RobotControl> buildTaskList() {
        try {
            SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
            delayString = prefs.getString(AutonomousOptions.START_DELAY_PREF, "0").replace(" sec", "");
            startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
            parkingRow = prefs.getString(AutonomousOptions.PARKING_PREF, AutonomousOptions.PARKING_LOCATION[0]).substring(4);
            Logger.logFile(AutonomousOptions.START_POS_MODES_PREF + " - " + startPosMode);
            Logger.logFile(AutonomousOptions.START_DELAY_PREF + " - " + delayString);
            Logger.logFile(AutonomousOptions.PARKING_PREF + " - " + parkingRow);
        }
        catch (Exception e) {
            RobotLog.e("SharedPref exception " + e);
            this.delayString = "0";
        }
        Logger.logFile("Done with init in autonomous");
        String postFix = (startPosMode.endsWith("RIGHT"))?"_RIGHT":"_LEFT";
        // 1. Start Delay
        if (!delayString.equals("0")) {
            taskList.add(new RobotSleep(Integer.parseInt(delayString) * 1000));
        }
        // 2. Close and Grab initial cone
        taskList.add(new GrabberTask(robotHardware, false));
        // 3. Lift to safe height
        taskList.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftSafeRotate));
        // 4. Rotate tullet to -90 degree
        Pose2d turretAngle = getProfilePose("TURRET_ANGLE_1" + postFix);
        taskList.add(new TurnTurretTask(robotHardware, (int)turretAngle.getX()));
        // 5. Move to first position
        Pose2d pos1 = getProfilePose("DROP_HIGH_1" + postFix);
        TrajectoryVelocityConstraint velConstraint;
        TrajectoryAccelerationConstraint accelConstraint;

        velConstraint = getVelocityConstraint(20, 15, robotProfile.hardwareSpec.trackWidth);
        accelConstraint = getAccelerationConstraint(15);
        // move to first line
        Pose2d p0 = new Pose2d(0,0,0);
        TrajectorySequence trj1 = drive.trajectorySequenceBuilder(p0)
                .lineTo(pos1.vec(), velConstraint, accelConstraint)
                .back(12)
                .turn(Math.PI/2)
                .forward(12)
                .build();
        SplineMoveTask moveToDrop1 = new SplineMoveTask(robotHardware.mecanumDrive, trj1);
        taskList.add(moveToDrop1);

        // Lift, Rotate and Drop
        Pose2d armHighPos = getProfilePose("DROP_HIGH_ARM" + postFix);
        taskList.add(new LiftArmTask(robotHardware, 3781));
        taskList.add(new TurnTurretTask(robotHardware, 0));
        taskList.add(new RobotSleep(1000));
        taskList.add(new LiftArmTask(robotHardware, 3281));
        taskList.add(new GrabberTask(robotHardware, true));
        // Done with drop
        taskList.add(new LiftArmTask(robotHardware, 3781)); // lift up to avoid pulling the yellow pole
        taskList.add(new TurnTurretTask(robotHardware, 925));
        taskList.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionDriverMin));
        taskList.add(new LiftArmTask(robotHardware, 520));
        TrajectorySequence trj2 = drive.trajectorySequenceBuilder(trj1.end())
                .back(34)
                .build();
        SplineMoveTask moveToPick1 = new SplineMoveTask(robotHardware.mecanumDrive, trj2);
        taskList.add(moveToPick1);
        // Doing #2
        taskList.add(new GrabberTask(robotHardware, false));
        taskList.add(new LiftArmTask(robotHardware,1000));  // lift before we can rotate
        taskList.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));
        TrajectorySequence trj3 = drive.trajectorySequenceBuilder(trj2.end())
                .forward(34)
                .build();
        SplineMoveTask moveToDrop2 = new SplineMoveTask(robotHardware.mecanumDrive, trj2);
        taskList.add(moveToDrop2);
        taskList.add(new LiftArmTask(robotHardware, 3781));
        taskList.add(new TurnTurretTask(robotHardware, 0));
        taskList.add(new RobotSleep(1000));
        taskList.add(new LiftArmTask(robotHardware, 3281));
        taskList.add(new GrabberTask(robotHardware, true));
        // Done with drop 2
        taskList.add(new LiftArmTask(robotHardware, 3781)); // lift up to avoid pulling the yellow pole
        taskList.add(new TurnTurretTask(robotHardware, 925));
        taskList.add(new LiftArmTask(robotHardware, 390));
        return taskList;
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
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