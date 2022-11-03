package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

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
    int parkingRow;
    AprilTagSignalRecognition aprilTagSignalRecognition;

    public AutonomousTaskBuilder(RobotHardware robotHardware, RobotProfile robotProfile, AprilTagSignalRecognition aprilTagSignalRecognition) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        this.aprilTagSignalRecognition = aprilTagSignalRecognition;
        drive = (NBMecanumDrive)robotHardware.getMecanumDrive();
    }

    public ArrayList<RobotControl> buildTaskList() {
        try {
            SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
            delayString = prefs.getString(AutonomousOptions.START_DELAY_PREF, "0").replace(" sec", "");
            startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
//            parkingRow = prefs.getString(AutonomousOptions.PARKING_PREF, AutonomousOptions.PARKING_LOCATION[0]).substring(4);
            parkingRow = aprilTagSignalRecognition.getRecognitionResult();
            Logger.logFile(AutonomousOptions.START_POS_MODES_PREF + " - " + startPosMode);
            Logger.logFile(AutonomousOptions.START_DELAY_PREF + " - " + delayString);
            Logger.logFile(AutonomousOptions.PARKING_PREF + " - " + parkingRow);
        }
        catch (Exception e) {
            RobotLog.e("SharedPref exception " + e);
            this.delayString = "0";
        }
        Logger.logFile("Done with init in autonomous");
        boolean isRight = startPosMode.endsWith("RIGHT");
        RobotProfile.AutonParam param = robotProfile.autonParam;
        TrajectoryVelocityConstraint velFast = getVelocityConstraint(param.fastVelocity, Math.toRadians(param.fastAngVelo), 3);
        TrajectoryAccelerationConstraint acceFast = getAccelerationConstraint(param.fastAcceleration);
        TrajectoryVelocityConstraint velConstraint = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), 3);
        TrajectoryAccelerationConstraint accelConstraint = getAccelerationConstraint(param.normAcceleration);

        // 1. Start Delay
        if (!delayString.equals("0")) {
            taskList.add(new RobotSleep(Integer.parseInt(delayString) * 1000));
        }
        // 2. Close and Grab initial cone
        taskList.add(new GrabberTask(robotHardware, false));
        // 3. Combo 1) Lift arm to high, 2) rotate tullet forward 3) move to right place
        ParallelComboTask initComb = new ParallelComboTask();
        initComb.add(new LiftArmTask(robotHardware, param.liftHighDrop));
        initComb.add(new TurnTurretTask(robotHardware, param.turretForwardPos));
        Pose2d p0 = new Pose2d(0,0,0);
        TrajectorySequence trj1 = drive.trajectorySequenceBuilder(p0)
                .lineTo(new Vector2d(param.forward1, 0), velConstraint, accelConstraint)
                .back(param.back1, velConstraint, accelConstraint)
                .build();
        SplineMoveTask moveToDrop1 = new SplineMoveTask(robotHardware.mecanumDrive, trj1);
        initComb.add(moveToDrop1);
        taskList.add(initComb);
        // 4. Comb 1) Rotate robot toward center, 2) rotate arm to drop position, 3) extend the arm
        ParallelComboTask dropComb1 = new ParallelComboTask();
        TrajectorySequence trjRotate = drive.trajectorySequenceBuilder(trj1.end())
                .turn((isRight)?Math.PI/2:-Math.PI/2)
                .build();
        SplineMoveTask rotate = new SplineMoveTask(robotHardware.mecanumDrive, trjRotate);
        dropComb1.add(rotate);
        dropComb1.add(new TurnTurretTask(robotHardware, (isRight)?param.turretDropPosRight:param.turretDropPosLeft));
        dropComb1.add(new ExtendArmTask(robotHardware, param.armLengthDrop));
        taskList.add(dropComb1);
        // sleep
        taskList.add(new RobotSleep(1000));
        // 1)lower 2) wait, open grab, retract, turn, move
        ParallelComboTask dropRetract1 = new ParallelComboTask();
            dropRetract1.add(new LiftArmTask(robotHardware, param.liftStack5));
            SequentialComboTask seq1 = new SequentialComboTask();
                seq1.add(new RobotSleep(300));
                seq1.add(new GrabberTask(robotHardware, true));
                seq1.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));
                seq1.add(new TurnTurretTask(robotHardware, (isRight)?param.turretPickPosRight:param.turretPickPosLeft));
                TrajectorySequence toPick = drive.trajectorySequenceBuilder(trjRotate.end())
                        .back(param.backPick)
                        .build();
                SplineMoveTask moveToPick1 = new SplineMoveTask(robotHardware.mecanumDrive, toPick);
                seq1.add(moveToPick1);
            dropRetract1.add(seq1);
        taskList.add(dropRetract1);
        // Doing #2 pick up
        taskList.add(new ExtendArmTask(robotHardware, param.armLengthPick));
        taskList.add(new RobotSleep(500));
        taskList.add(new GrabberTask(robotHardware, false));
        taskList.add(new LiftArmTask(robotHardware,param.liftUpSafe));  // lift before we can rotate
        // Comb 1) Lift to high, rotate, move forward
        ParallelComboTask dropComb2 = new ParallelComboTask();
            dropComb2.add(new LiftArmTask(robotHardware,param.liftHighDrop));  // lift before we can rotate
            dropComb2.add(new TurnTurretTask(robotHardware, (isRight)?param.turretDropPosRight:param.turretDropPosLeft));
            TrajectorySequence toDrop2 = drive.trajectorySequenceBuilder(toPick.end())
                    .forward(param.backPick)
                    .build();
            SplineMoveTask moveToDrop2 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop2);
            dropComb2.add(moveToDrop2);
        taskList.add(dropComb2);
        // extend arm
        taskList.add(new ExtendArmTask(robotHardware, param.armLengthDrop));
        taskList.add(new RobotSleep(1000));
        // 1)lower 2) wait, open grab, retract, turn, move
        ParallelComboTask dropRetract2 = new ParallelComboTask();
            dropRetract2.add(new LiftArmTask(robotHardware, param.liftStack4));
            SequentialComboTask seq2 = new SequentialComboTask();
                seq2.add(new RobotSleep(300));
                seq2.add(new GrabberTask(robotHardware, true));
                seq2.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));
                seq2.add(new TurnTurretTask(robotHardware, (isRight)?param.turretPickPosRight:param.turretPickPosLeft));
                TrajectorySequence toParkTrj = drive.trajectorySequenceBuilder(toDrop2.end())
                        .back(param.backPick)
                        .build();
                SplineMoveTask toPark = new SplineMoveTask(robotHardware.mecanumDrive, toParkTrj);
            seq2.add(toPark);
            dropRetract2.add(seq2);
        taskList.add(dropRetract2);
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