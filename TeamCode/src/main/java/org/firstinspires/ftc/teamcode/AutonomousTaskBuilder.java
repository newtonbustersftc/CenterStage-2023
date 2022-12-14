package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

//almost worked version for high pole first and then low pole and possible ground junction
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
    boolean isRed = false;

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
        isRed = startPosMode.contains("RED");
        RobotProfile.AutonParam param = robotProfile.autonParam;
        TrajectoryVelocityConstraint velFast = getVelocityConstraint(param.fastVelocity, Math.toRadians(param.fastAngVelo), 3);
        TrajectoryAccelerationConstraint acceFast = getAccelerationConstraint(param.fastAcceleration);
        TrajectoryVelocityConstraint velConstraint = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), 3);
        TrajectoryAccelerationConstraint accelConstraint = getAccelerationConstraint(param.normAcceleration);
        TrajectoryVelocityConstraint velSlow = getVelocityConstraint((param.normVelocity)/2, Math.toRadians((param.normAngVelo)/2), 3);
        TrajectoryAccelerationConstraint acceSlow = getAccelerationConstraint((param.normAcceleration)/2);

        // 1. Start Delay
        if (!delayString.equals("0")) {
            taskList.add(new RobotSleep(Integer.parseInt(delayString) * 1000));
        }
        // 2. Close and Grab initial cone
        taskList.add(new GrabberTask(robotHardware, false));
        // 3. Combo 1) Lift arm to high, 2) rotate tullet forward 3) move to right place
        ParallelComboTask initComb = new ParallelComboTask();
        initComb.add(new LiftArmTask(robotHardware,param.liftMidDrop ));//it's mid pole
        initComb.add(new TurnTurretTask(robotHardware, (isRight) ? -1458: -455)); //param.turretDropPosLeft
        Pose2d p0 = new Pose2d(0,0,0);
        TrajectorySequence trj1 = drive.trajectorySequenceBuilder(p0)
                .lineTo(new Vector2d(param.forward1, 0), velConstraint, accelConstraint)  //param.forward1, 54 high, 32 mid,
                .build();
        SplineMoveTask moveToDrop1 = new SplineMoveTask(robotHardware.mecanumDrive, trj1);
        initComb.add(moveToDrop1);
        initComb.add(new ExtendArmTask(robotHardware, param.armLengthDrop1));
        taskList.add(initComb);
        // sleep
        taskList.add(new RobotSleep(300)); //test:robot stay here longer than 500ms
//         1)lower 2) wait, open grab, retract, turn, move
        ParallelComboTask dropRetract1 = new ParallelComboTask();
        dropRetract1.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftPickPos[4]));
        SequentialComboTask seq1 = new SequentialComboTask();
        seq1.add(new RobotSleep(150));
        seq1.add(new GrabberTask(robotHardware, true));

        ParallelComboTask toPickUpStackCone = new ParallelComboTask();
        toPickUpStackCone.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));
        toPickUpStackCone.add(new TurnTurretTask(robotHardware, (isRight) ? 0 : -1850));//-2775 param.turretPickPosRight:param.turretPickPosLeft
        TrajectorySequence toPick ;
        if(isRight){
            toPick = drive.trajectorySequenceBuilder(trj1.end())
                    .lineTo(new Vector2d(57, 0), velConstraint, accelConstraint)  //param.forward1, 56 high, 41.5 mid,
                    .back(4, velConstraint, accelConstraint)
                    .strafeRight(param.pickStraftRight)
                    .build();
        }else {
            toPick = drive.trajectorySequenceBuilder(trj1.end())
                    .lineTo(new Vector2d(57, 0), velConstraint, accelConstraint)  //param.forward1, 56 high, 41.5 mid,
                    .back(3, velConstraint, accelConstraint)
                    .strafeLeft(param.pickStraftLeft)
                    .build();
        }
        SplineMoveTask moveToPick1 = new SplineMoveTask(robotHardware.mecanumDrive, toPick);
        toPickUpStackCone.add(moveToPick1);
        seq1.add(toPickUpStackCone);
        dropRetract1.add(seq1);
        taskList.add(dropRetract1);
        taskList.add(new ExtendArmTask(robotHardware, (isRight) ? param.armLengthPickRight: param.armLengthPickLeft, 200));

        // Doing #1 pick up from stack
        SequentialComboTask pickup_delivery2 = new SequentialComboTask();
        pickup_delivery2.add(new GrabberTask(robotHardware, false));
        pickup_delivery2.add(new RobotSleep(200));

        //lift up right will hit the wall, retrieve with an angle
        ParallelComboTask liftAndRetrieve = new ParallelComboTask();
        liftAndRetrieve.add(new ExtendArmTask(robotHardware, param.armLengthPostPick));
        liftAndRetrieve.add(new LiftArmTask(robotHardware,param.liftUpSafe)); //lift up only a height of a cone
        pickup_delivery2.add(liftAndRetrieve);  // lift before we can rotate

        // Drop to low pole
        ParallelComboTask dropComb2 = new ParallelComboTask();
        dropComb2.add(new LiftArmTask(robotHardware,param.liftLowDrop));
        dropComb2.add(new TurnTurretTask(robotHardware, (isRight)?1297:-3147));
        dropComb2.add(new ExtendArmTask(robotHardware, param.armLengthDropLow));
        pickup_delivery2.add(dropComb2);
        taskList.add(pickup_delivery2);

        // 1)lower 2) wait, open grab, retract, turn, move
        ParallelComboTask dropRetract2 = new ParallelComboTask();
        dropRetract2.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftPickPos[3]));
        SequentialComboTask seq2 = new SequentialComboTask();
        seq2.add(new RobotSleep(150));
        seq2.add(new GrabberTask(robotHardware, true));
        dropRetract2.add(seq2);
        taskList.add(dropRetract2);

        // Lift and go to pick up #2 from stack
        ParallelComboTask upRetract = new ParallelComboTask();
        upRetract.add(new LiftArmTask(robotHardware, param.liftLowDrop));
        upRetract.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.INIT));
//        upRetract.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionFullInPos));
        upRetract.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionInitPos));
        upRetract.add(new TurnTurretTask(robotHardware, (isRight)? 0 :-1850));//param.turretDropPosRight,param.turretDropPosLeft
        taskList.add(upRetract);

        ParallelComboTask downExtendGrab = new ParallelComboTask();
        downExtendGrab.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftPickPos[3], true));
        downExtendGrab.add(new GrabberTask(robotHardware, true));
        downExtendGrab.add(new ExtendArmTask(robotHardware, (isRight) ? param.armLengthPickRight: param.armLengthPickLeft, 200));
        taskList.add(downExtendGrab);

        // Doing #2 pick up from stack
        SequentialComboTask seqPick2 = new SequentialComboTask();
//        seqPick2.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftPickPos[3]));
        seqPick2.add(new GrabberTask(robotHardware, false));
        seqPick2.add(new RobotSleep(200));//400
        taskList.add(seqPick2);

        // Deliver to ground junction
        SequentialComboTask delivery3 = new SequentialComboTask();
        //lift up right will hit the wall, retrieve with an angle
        ParallelComboTask liftAndRetrieve2 = new ParallelComboTask();
        liftAndRetrieve2.add(new ExtendArmTask(robotHardware, param.armLengthPostPick));
        liftAndRetrieve2.add(new LiftArmTask(robotHardware,param.liftUpSafe)); // param.liftUpSafe lift up only a height of a cone
        delivery3.add(liftAndRetrieve2);

        ParallelComboTask dropComb3 = new ParallelComboTask();
        dropComb3.add(new TurnTurretTask(robotHardware, isRight ? param.turretGroundRight:param.turretGroundLeft));//-1470
        dropComb3.add(new ExtendArmTask(robotHardware, param.armLengthDropGround));
        delivery3.add(dropComb3);
        taskList.add(delivery3);
//        taskList.add(new ExtendArmTask(robotHardware, param.armLengthDrop-0.05));
//        taskList.add(new RobotSleep(100));

        // 1)lower 2) wait, open grab, retract, turn, move
        SequentialComboTask dropGroundJunction = new SequentialComboTask();
        dropGroundJunction.add(new LiftArmTask(robotHardware,param.liftGroundDrop));
        dropGroundJunction.add(new RobotSleep(50));
        dropGroundJunction.add(new GrabberTask(robotHardware, true));
        taskList.add(dropGroundJunction);

        ParallelComboTask liftRetract3 = new ParallelComboTask();
        liftRetract3.add(new LiftArmTask(robotHardware, param.liftUpSafe));
        liftRetract3.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionFullInPos));
        taskList.add(liftRetract3);

        //final
        parkingRow = aprilTagSignalRecognition.getRecognitionResult();
        if ((isRight && parkingRow==3) || (!isRight && parkingRow==1)) {
            // no parking, pick up and drop to low pole
            // Doing #3 pick up from stack
            taskList.add(new TurnTurretTask(robotHardware, (isRight)? 0 :-1850));//param.turretDropPosRight,param.turretDropPosLeft
            ParallelComboTask thirdPickup = new ParallelComboTask();
            thirdPickup.add(new ExtendArmTask(robotHardware, (isRight) ? param.armLengthPickRight: param.armLengthPickLeft, 200));
            thirdPickup.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftPickPos[2], true));
            taskList.add(thirdPickup);
            SequentialComboTask seqPick3 = new SequentialComboTask();
            seqPick3.add(new GrabberTask(robotHardware, false));
            seqPick3.add(new RobotSleep(200));
            taskList.add(seqPick3);

            //lift up right will hit the wall, retrieve with an angle
            ParallelComboTask liftAndRetrieve3 = new ParallelComboTask();
            liftAndRetrieve3.add(new ExtendArmTask(robotHardware, param.armLengthPostPick));
            liftAndRetrieve3.add(new LiftArmTask(robotHardware,param.liftUpSafe)); //lift up only a height of a cone
            taskList.add(liftAndRetrieve3);

            ParallelComboTask rotateLowPole = new ParallelComboTask();
            rotateLowPole.add(new LiftArmTask(robotHardware, param.liftLowDrop));
            rotateLowPole.add(new TurnTurretTask(robotHardware,  (isRight)?1297:-3147));
            rotateLowPole.add(new ExtendArmTask(robotHardware, param.armLengthDropLow));
            taskList.add(rotateLowPole);

            ParallelComboTask dropLowPole = new ParallelComboTask();
            dropLowPole.add(new LiftArmTask(robotHardware, robotProfile.hardwareSpec.liftPickPos[4]));
            SequentialComboTask seqOpen = new SequentialComboTask();
//                seqParking.add(new RobotSleep(300)); ??
            seqOpen.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.INIT));
            seqOpen.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionFullInPos));
            ParallelComboTask finalPos = new ParallelComboTask();
            finalPos.add(new GrabberTask(robotHardware, false));
            finalPos.add(new TurnTurretTask(robotHardware,  (isRight)?1097:-2900));
            seqOpen.add(finalPos);
            dropLowPole.add(seqOpen);
            taskList.add(dropLowPole);
        }
        else {
            ParallelComboTask parkingComb = new ParallelComboTask();
            TrajectorySequence parking;
            if(isRight){
                parking = drive.trajectorySequenceBuilder(toPick.end())
                        .strafeLeft((parkingRow==2)?24:48)   //y=54
                        .build();
            }else {
                parking = drive.trajectorySequenceBuilder(toPick.end())
                        .strafeRight((parkingRow==2)?24:48)   //y=54
                        .build();
            }
            SplineMoveTask moveToPakring = new SplineMoveTask(robotHardware.mecanumDrive, parking);
            parkingComb.add(moveToPakring);
            parkingComb.add(new TurnTurretTask(robotHardware, 0));
            taskList.add(parkingComb);
            taskList.add(new TurnTurretTask(robotHardware, 925));
        }
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
