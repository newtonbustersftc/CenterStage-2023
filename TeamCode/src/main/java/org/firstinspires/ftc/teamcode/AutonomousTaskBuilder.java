package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.text.InputFilter;

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
import java.util.Vector;

//almost worked version for high pole first and then low pole and possible ground junction
public class AutonomousTaskBuilder {
    RobotProfile robotProfile;
    RobotHardware robotHardware;
    ArrayList<RobotControl> taskList = new ArrayList<>();
    NBMecanumDrive drive;
    String delayString;
    String startPosMode;
    int parkingRow;
    AprilTagSignalRecognition aprilTagSignalRecognition;
    String aimPole;
    PoleRecognition poleRecognition;

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
            parkingRow = aprilTagSignalRecognition.getRecognitionResult();
            aimPole = prefs.getString(AutonomousOptions.AIM_POLES_PREF, AutonomousOptions.AIM_POLES[0]);
            Logger.logFile(AutonomousOptions.START_POS_MODES_PREF + " - " + startPosMode);
            Logger.logFile(AutonomousOptions.START_DELAY_PREF + " - " + delayString);
            Logger.logFile(AutonomousOptions.PARKING_PREF + " - " + parkingRow);
            Logger.logFile(AutonomousOptions.AIM_POLES_PREF + " - " + aimPole);
        } catch (Exception e) {
            RobotLog.e("SharedPref exception " + e);
            this.delayString = "0";
        }
        Logger.logFile("aim pole:"+aimPole);
        Logger.logFile("Done with init in autonomous");
        boolean isRight = startPosMode.endsWith("RIGHT");
        boolean isHighPole = aimPole.contains("HIGH");
        RobotProfile.AutonParam param = robotProfile.autonParam;
        TrajectoryVelocityConstraint velFast = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), robotProfile.hardwareSpec.trackWidth);
        TrajectoryAccelerationConstraint accelFast = getAccelerationConstraint(param.fastAcceleration);
        TrajectoryVelocityConstraint velConstraint = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), 3);
        TrajectoryAccelerationConstraint accelConstraint = getAccelerationConstraint(param.normAcceleration);

        Vector2d rightStack =new Vector2d(49.497,21.21);//49.497, 21.21
        Vector2d leftStack = new Vector2d(21.21, 49.497);
        Vector2d rightStack_parking =new Vector2d(50.2,20.5);//51.6, 19.1
        Vector2d leftStack_parking = new Vector2d(17.5, 51);//19.1, 51.6
        Vector2d leftFar_parking3 = new Vector2d(51.62,19.09);//52.33,18.38
        Vector2d rightFar_parking1 = new Vector2d(18.38,52.33);

        double robHead_left = -0.7863025080592201;//-45
        double robHead_right = -2.35619;  //-135

        SoloDriverOpMode.LastLiftExtTut beginLiftExtTut_high = new SoloDriverOpMode().new LastLiftExtTut();
        beginLiftExtTut_high.liftPos = param.liftHighDrop;
        beginLiftExtTut_high.extension = isRight? param.armLengthRightDrop1 : param.armLengthLeftDrop1 + 0.05; //0.65
        beginLiftExtTut_high.tutPos = isRight ? param.turretDropPosRightHighPole : param.turretDropPosLeftHighPole;    //-70
        beginLiftExtTut_high.robHead = isRight? robHead_right : robHead_left;

        SoloDriverOpMode.LastLiftExtTut beginLiftExtTut_mid = new SoloDriverOpMode().new LastLiftExtTut();
        beginLiftExtTut_mid.liftPos = param.liftMidDrop;
        beginLiftExtTut_mid.extension = isRight? param.armLengthRightDrop1 : param.armLengthLeftDrop1-0.1; //0.65
        beginLiftExtTut_mid.tutPos = isRight ? param.turretDropPosRightMidPole_begin : param.turretDropPosLeftMidPole_begin;   //-15
        beginLiftExtTut_mid.robHead = isRight? robHead_right : robHead_left;;

        SoloDriverOpMode.LastLiftExtTut liftExtTut_2 = new SoloDriverOpMode().new LastLiftExtTut();
        liftExtTut_2.liftPos = robotProfile.hardwareSpec.liftPickPos[4];
        liftExtTut_2.extension = isRight ? param.armLengthPickRight: param.armLengthPickLeft ;
        liftExtTut_2.tutPos = isRight ?  1385 : -1460;
        liftExtTut_2.robHead = isRight? robHead_right : robHead_left;

        SoloDriverOpMode.LastLiftExtTut liftExtTut_3 = new SoloDriverOpMode().new LastLiftExtTut();
        liftExtTut_3.liftPos = robotProfile.hardwareSpec.liftPickPos[3];
        liftExtTut_3.extension = isRight ? param.armLengthPickRight : param.armLengthPickLeft ;
        liftExtTut_3.tutPos = isRight ? 1385 : -1460 ;
        liftExtTut_3.robHead = isRight? robHead_right : robHead_left;

        SoloDriverOpMode.LastLiftExtTut liftExtTut_4 = new SoloDriverOpMode().new LastLiftExtTut();
        liftExtTut_4.liftPos = robotProfile.hardwareSpec.liftPickPos[2];
        liftExtTut_4.extension = isRight ? param.armLengthPickRight : param.armLengthPickLeft;
        liftExtTut_4.tutPos = isRight ? 1385 : -1440;
        liftExtTut_4.robHead = isRight? robHead_right : robHead_left;

        SoloDriverOpMode.LastLiftExtTut liftExtTut_5 = new SoloDriverOpMode().new LastLiftExtTut();
        liftExtTut_5.liftPos = robotProfile.hardwareSpec.liftPickPos[1];
        liftExtTut_5.extension = isRight ? param.armLengthPickRight : param.armLengthPickLeft;
        liftExtTut_5.tutPos = isRight ? 1385 : -1440;
        liftExtTut_5.robHead = isRight? robHead_right : robHead_left;

        SoloDriverOpMode.LastLiftExtTut liftExtTut_6 = new SoloDriverOpMode().new LastLiftExtTut();
        liftExtTut_6.liftPos = robotProfile.hardwareSpec.liftPickPos[4];
        liftExtTut_6.extension = isRight ? param.armLengthPickRight : param.armLengthPickLeft;
        liftExtTut_6.tutPos = isRight ? 1385 : -1850;
        liftExtTut_6.robHead = isRight? robHead_right : robHead_left;

        SoloDriverOpMode.LastLiftExtTut centralLiftExtTut_high = new SoloDriverOpMode().new LastLiftExtTut();
        centralLiftExtTut_high.liftPos = param.liftHighDrop;
        centralLiftExtTut_high.extension = isRight ? param.armLengthRightDrop1 : param.armLengthLeftDrop1 + 0.05;
        centralLiftExtTut_high.tutPos = isRight ? -30 : param.turretDropPosLeftHighPole; //param.turretDropPosRightHighPole
        centralLiftExtTut_high.robHead = isRight? robHead_right : robHead_left;

        SoloDriverOpMode.LastLiftExtTut centralLiftExtTut_mid = new SoloDriverOpMode().new LastLiftExtTut();
        centralLiftExtTut_mid.liftPos = param.liftMidDrop;
        centralLiftExtTut_mid.extension = isRight ? param.armLengthLeftDrop1 : param.armLengthLeftDrop1-0.1;
        centralLiftExtTut_mid.tutPos = isRight ? param.turretDropPosRightMidPole_deliver : param.turretDropPosLeftMidPole_deliver;
        centralLiftExtTut_mid.robHead = isRight? robHead_right : robHead_left;

        //this is a transition task, after pick up cone, lift up cone in an angle, no change is need here
        SoloDriverOpMode.LastLiftExtTut pickUpConeLiftExtTut = new SoloDriverOpMode().new LastLiftExtTut();
        pickUpConeLiftExtTut.liftPos = param.liftUpSafe;
        pickUpConeLiftExtTut.extension = param.armLengthPostPick;
        pickUpConeLiftExtTut.tutPos = isRight ? 1385 : -1385;
        pickUpConeLiftExtTut.robHead = isRight? robHead_right : robHead_left;

        //arrived at the final position, turn and retract before stop
        //apply to both left parking1 and right parking 3,this is final position in parking
        SoloDriverOpMode.LastLiftExtTut finalLiftExtTut_parkingNearStack = new SoloDriverOpMode().new LastLiftExtTut();
        finalLiftExtTut_parkingNearStack.liftPos = robotProfile.hardwareSpec.liftPickPos[4];
        finalLiftExtTut_parkingNearStack.extension = robotProfile.hardwareSpec.extensionInitPos;
        finalLiftExtTut_parkingNearStack.tutPos = isRight ? 1385 :-1850;//-1385
        finalLiftExtTut_parkingNearStack.robHead = isRight? robHead_right : robHead_left;

        //arrived at the final position, turn and retract before stop
        //apply to both left parking2 and right parking2,this is final position in parking
        SoloDriverOpMode.LastLiftExtTut finalLiftExtTut_parking2 = new SoloDriverOpMode().new LastLiftExtTut();
        finalLiftExtTut_parking2.liftPos = robotProfile.hardwareSpec.liftPickPos[4];
        finalLiftExtTut_parking2.extension = param.armLengthPostPick;
//        finalLiftExtTut_parking2.tutPos = isRight ? isHighPole ? -1385:-460 : 1385;  //mid number-460?
        finalLiftExtTut_parking2.tutPos = isRight ?  -1385 : 1385;  //mid number-460?
        finalLiftExtTut_parking2.robHead = isRight? robHead_right : robHead_left;

        //arrived at the final position, turn and retract before stop
        //apply to both left parking 3 and right parking 1, this is final position in parking
        SoloDriverOpMode.LastLiftExtTut finalLiftExtTut_parkingFarStack = new SoloDriverOpMode().new LastLiftExtTut();
        finalLiftExtTut_parkingFarStack.liftPos = robotProfile.hardwareSpec.liftPickPos[4];
        finalLiftExtTut_parkingFarStack.extension = param.armLengthPostPick;
        finalLiftExtTut_parkingFarStack.tutPos = isRight ? -1385 : -2300; //right: -1385
        finalLiftExtTut_parkingFarStack.robHead = isRight? robHead_right : robHead_left;

        //final dropping locations for far distance => left parking3 or right parking 1
        //Dropping the last cone from different zone: apply to ONLY to left side parking 3 and right side parking 1
        SoloDriverOpMode.LastLiftExtTut parkingCentralLiftExtTut_high = new SoloDriverOpMode().new LastLiftExtTut();
        parkingCentralLiftExtTut_high.liftPos = param.liftHighDrop;
        parkingCentralLiftExtTut_high.extension = isRight? param.armLengthRightDrop1-0.2 : param.armLengthLeftDrop1+0.2;
        parkingCentralLiftExtTut_high.tutPos =isRight ? param.turretRight1FinalPos : param.turretLeft3FinalPos ;//1318=925*2-925-70*2
        parkingCentralLiftExtTut_high.robHead = isRight? robHead_right : robHead_left;

        SoloDriverOpMode.LastLiftExtTut parkingCentralLiftExtTut_mid = new SoloDriverOpMode().new LastLiftExtTut();
        parkingCentralLiftExtTut_mid.liftPos = param.liftMidDrop;
        parkingCentralLiftExtTut_mid.extension = param.armLengthLeftDrop1;
        parkingCentralLiftExtTut_mid.tutPos =isRight ?  param.turretLeft1FinalPos : -1870;//1318=925*2-925-70*2
        parkingCentralLiftExtTut_mid.robHead = isRight? robHead_right : robHead_left;

        Vector2d deliveryCenter = new Vector2d(param.forward / Math.sqrt(2), param.forward / Math.sqrt(2));

        // 1. Start Delay
        if (!delayString.equals("0")) {
            taskList.add(new RobotSleep(Integer.parseInt(delayString) * 1000));
        }

        // picking and deliver #1 cone, close and grab initial cone
        taskList.add(new GrabberTask(robotHardware, false));

        ParallelComboTask initComb = new ParallelComboTask();
        Pose2d p0 = new Pose2d(0, 0, 0);
        TrajectorySequence toDrop_1 = drive.trajectorySequenceBuilder(p0)
                .lineTo(deliveryCenter, velConstraint, accelConstraint)
                .build();
        SplineMoveTask moveToDrop1 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop_1);
        initComb.add(moveToDrop1);
        initComb.add(new DrvOpLiftExtTutTask(robotHardware, isHighPole ? beginLiftExtTut_high : beginLiftExtTut_mid,  true)); //0 for mid pole, rise the lift later
        taskList.add(initComb);
        taskList.add(new RobotSleep(200)); //??test:robot stay here longer than 500ms

        ParallelComboTask dropRetract2 = new ParallelComboTask();
        dropRetract2.add(new LiftArmTask(robotHardware, isHighPole ? 700 : 450)); //robotProfile.hardwareSpec.liftPickPos[4])
        SequentialComboTask seq2 = new SequentialComboTask();
        seq2.add(new RobotSleep(200));
        seq2.add(new GrabberTask(robotHardware, true));

        // picking #2-#5 cone (#1-#4 from stack)
        ParallelComboTask comboToPick_2 = new ParallelComboTask();
//        comboToPick_2.add(new LiftExtTutTask(robotHardware, liftExtTut_2, 1));
        comboToPick_2.add(new DrvOpLiftExtTutTask(robotHardware, liftExtTut_2, true));

        TrajectorySequence toPick_2;
        toPick_2 = drive.trajectorySequenceBuilder(toDrop_1.end())
                .lineTo(isRight ? rightStack : leftStack, velFast, accelFast)
                .build();

        SplineMoveTask moveToPick2 = new SplineMoveTask(robotHardware.mecanumDrive, toPick_2);
        comboToPick_2.add(moveToPick2);
        seq2.add(comboToPick_2);

        seq2.add(new GrabberTask(robotHardware, false));
        seq2.add(new RobotSleep(100));
        seq2.add(new LiftExtTutTask(robotHardware, pickUpConeLiftExtTut, 2));

        ParallelComboTask dropComb2 = new ParallelComboTask();
//        forward = param.forward_high;
        TrajectorySequence toDrop_2 = drive.trajectorySequenceBuilder(toPick_2.end())
                .lineTo( deliveryCenter, velFast, accelFast)  //param.forward1/Math.sqrt(2), mid 32
                .build();
        SplineMoveTask moveToDrop_2 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop_2);
        dropComb2.add(moveToDrop_2);
        //dropComb2.add(new LiftExtTutTask(robotHardware, isHighPole ? centralLiftExtTut_high : centralLiftExtTut_mid, 1));
        dropComb2.add(new DrvOpLiftExtTutTask(robotHardware, isHighPole ? centralLiftExtTut_high : centralLiftExtTut_mid, true));
        seq2.add(dropComb2);
        dropRetract2.add(seq2);
        taskList.add(dropRetract2);
        taskList.add(new RobotSleep(150));

        ParallelComboTask dropRetract3 = new ParallelComboTask();
        dropRetract3.add(new LiftArmTask(robotHardware, isHighPole ? 700 : 450));
        SequentialComboTask seq3 = new SequentialComboTask();
        seq3.add(new RobotSleep(200));
        seq3.add(new GrabberTask(robotHardware, true));

        // picking #3 cone (#2 from stack)
        ParallelComboTask comboToPick_3 = new ParallelComboTask();
        comboToPick_3.add(new DrvOpLiftExtTutTask(robotHardware, liftExtTut_3, true));

        TrajectorySequence toPick_3;
        toPick_3 = drive.trajectorySequenceBuilder(toDrop_2.end())
                .lineTo(isRight ? rightStack : leftStack, velFast, accelFast)
                .build();

        SplineMoveTask moveToPick3 = new SplineMoveTask(robotHardware.mecanumDrive, toPick_3);
        comboToPick_3.add(moveToPick3);

        seq3.add(comboToPick_3);
        seq3.add(new GrabberTask(robotHardware, false));
        seq3.add(new RobotSleep(100));
        seq3.add(new LiftExtTutTask(robotHardware, pickUpConeLiftExtTut, 2));

        ParallelComboTask dropComb3 = new ParallelComboTask();
        TrajectorySequence toDrop_3 = drive.trajectorySequenceBuilder(toPick_3.end())
                .lineTo(deliveryCenter, velFast, accelFast)  //param.forward1/Math.sqrt(2), mid 32
                .build();
        SplineMoveTask moveToDrop_3 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop_3);
        dropComb3.add(moveToDrop_3);
        dropComb3.add(new DrvOpLiftExtTutTask(robotHardware, isHighPole ? centralLiftExtTut_high: centralLiftExtTut_mid,true));
        seq3.add(dropComb3);
        dropRetract3.add(seq3);
        taskList.add(dropRetract3);
        taskList.add(new RobotSleep(150));

        ParallelComboTask dropRetract4 = new ParallelComboTask();
        dropRetract4.add(new LiftArmTask(robotHardware, isHighPole?700:450));
        SequentialComboTask seq4 = new SequentialComboTask();
        seq4.add(new RobotSleep(200));
        seq4.add(new GrabberTask(robotHardware, true));

        // picking #4 cone (#3 from stack)
        ParallelComboTask comboToPick_4 = new ParallelComboTask();
        comboToPick_4.add(new DrvOpLiftExtTutTask(robotHardware, liftExtTut_4, true));
        TrajectorySequence toPick_4;
        toPick_4 = drive.trajectorySequenceBuilder(toDrop_3.end())
                .lineTo(isRight ? rightStack : leftStack, velFast, accelFast)
                .build();

        SplineMoveTask moveToPick4 = new SplineMoveTask(robotHardware.mecanumDrive, toPick_4);
        comboToPick_4.add(moveToPick4);

        seq4.add(comboToPick_4);
        seq4.add(new GrabberTask(robotHardware, false));
        seq4.add(new RobotSleep(100));
        seq4.add(new LiftExtTutTask(robotHardware, pickUpConeLiftExtTut, 2));

        ParallelComboTask dropComb4 = new ParallelComboTask();
        TrajectorySequence toDrop_4 = drive.trajectorySequenceBuilder(toPick_4.end())
                .lineTo(deliveryCenter, velFast, accelFast)  //param.forward1/Math.sqrt(2), mid 32
                .build();
        SplineMoveTask moveToDrop_4 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop_4);
        dropComb4.add(moveToDrop_4);

        dropComb4.add(new DrvOpLiftExtTutTask(robotHardware, isHighPole ? centralLiftExtTut_high : centralLiftExtTut_mid, true));
        seq4.add(dropComb4);
        dropRetract4.add(seq4);
        taskList.add(dropRetract4);
        taskList.add(new RobotSleep(150));

        ParallelComboTask dropRetract5 = new ParallelComboTask();
        dropRetract5.add(new LiftArmTask(robotHardware, isHighPole ? 700 : 450));
        SequentialComboTask seq5 = new SequentialComboTask();
        seq5.add(new RobotSleep(200));
        seq5.add(new GrabberTask(robotHardware, true));

//        seq5.add(new AutoConePlacementTask(robotHardware, robotProfile));

        // picking #5 cone (#4 from stack)
        ParallelComboTask comboToPick_5 = new ParallelComboTask();
        comboToPick_5.add(new DrvOpLiftExtTutTask(robotHardware, liftExtTut_5, true));
        TrajectorySequence toPick_5;
        toPick_5 = drive.trajectorySequenceBuilder(toDrop_4.end())
                .lineTo((isRight) ? rightStack : leftStack, velFast, accelFast)
                .build();
        SplineMoveTask moveToPick5 = new SplineMoveTask(robotHardware.mecanumDrive, toPick_5);
        comboToPick_5.add(moveToPick5);

        seq5.add(comboToPick_5);
        seq5.add(new GrabberTask(robotHardware, false));
        seq5.add(new RobotSleep(100));
        seq5.add(new LiftExtTutTask(robotHardware, pickUpConeLiftExtTut, 2));

        ParallelComboTask dropComb5 = new ParallelComboTask();
        //apply to both left parking1 & parking2 + right parking2 & parking3
        TrajectorySequence toDrop_5_Parking1_Parking2 = drive.trajectorySequenceBuilder(toPick_5.end())
                .lineTo(deliveryCenter, velFast, accelFast)  //param.forward1/Math.sqrt(2), mid 32
//                .lineTo(new Vector2d(param.forward1 / Math.sqrt(2)+1/Math.sqrt(2), param.forward1 / Math.sqrt(2)-1/Math.sqrt(2)), velFast, accelFast)  //ending right 1"
//                .lineTo(new Vector2d(param.forward1 / Math.sqrt(2)-1/Math.sqrt(2), param.forward1 / Math.sqrt(2)-1/Math.sqrt(2)), velFast, accelFast)  //ending back 1"
//                .lineTo(new Vector2d(param.forward1 / Math.sqrt(2), param.forward1 / Math.sqrt(2)-Math.sqrt(2)), velFast, accelFast)  //ending right 1" and back 1"
                .build();
        //apply to left parking3 and right parking1
        TrajectorySequence toDrop_5_Parking3 = drive.trajectorySequenceBuilder(toPick_5.end())
                .lineTo(isRight ? rightFar_parking1 : leftFar_parking3, velFast, accelFast)
                .build();
        SplineMoveTask moveToDrop_5 = null;

        //final delivery position
        if(!isRight) { //left side
            if (parkingRow == 1 || parkingRow == 2) {
                moveToDrop_5 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop_5_Parking1_Parking2);
                dropComb5.add(moveToDrop_5);
                dropComb5.add(new DrvOpLiftExtTutTask(robotHardware, isHighPole ? centralLiftExtTut_high : centralLiftExtTut_mid, true));
            } else { //parkingRow == 3
                moveToDrop_5 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop_5_Parking3);
                dropComb5.add(moveToDrop_5);
                dropComb5.add(new DrvOpLiftExtTutTask(robotHardware, isHighPole ? parkingCentralLiftExtTut_high : parkingCentralLiftExtTut_mid, true));
            }
        }else{  //right side
            if (parkingRow == 2 || parkingRow == 3) {
                moveToDrop_5 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop_5_Parking1_Parking2);
                dropComb5.add(moveToDrop_5);
                dropComb5.add(new DrvOpLiftExtTutTask(robotHardware, isHighPole ? centralLiftExtTut_high : centralLiftExtTut_mid, true));
            } else { //parkingRow == 1
                moveToDrop_5 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop_5_Parking3);
                dropComb5.add(moveToDrop_5);
                dropComb5.add(new DrvOpLiftExtTutTask(robotHardware, isHighPole ? parkingCentralLiftExtTut_high : parkingCentralLiftExtTut_mid, true));
            }
        }

        seq5.add(dropComb5);
        dropRetract5.add(seq5);
        taskList.add(dropRetract5);
        taskList.add(new RobotSleep(200));

        ParallelComboTask dropRetract6 = new ParallelComboTask();
        dropRetract6.add(new LiftArmTask(robotHardware, isHighPole?800:500));
        SequentialComboTask seq6 = new SequentialComboTask();
        seq6.add(new RobotSleep(200));
        seq6.add(new GrabberTask(robotHardware, true));

        // picking #6 cone (#5 from stack)
//        ParallelComboTask comboToPick_6 = new ParallelComboTask();
//        comboToPick_6.add(new LiftExtTutTask(robotHardware, liftExtTut_6, 1));
//        TrajectorySequence toPick_6;
//        if (isRight) {
//            toPick_6 = drive.trajectorySequenceBuilder(toDrop_5.end())
//                    .lineTo(new Vector2d(21.21, 49.497), velFast, accelFast)
//                    .build();
//        } else {
//            toPick_6 = drive.trajectorySequenceBuilder(toDrop_5.end())
//                    .lineTo(new Vector2d(21.21, 49.497), velFast, accelFast)
//                    .build();
//        }
//        SplineMoveTask moveToPick6 = new SplineMoveTask(robotHardware.mecanumDrive, toPick_6);
//        comboToPick_6.add(moveToPick6);
//
//        seq6.add(comboToPick_6);
//        seq6.add(new GrabberTask(robotHardware, false));
//        seq6.add(new RobotSleep(150));
//        seq6.add(new LiftExtTutTask(robotHardware, pickUpConeLiftExtTut, 2));

//        ParallelComboTask dropComb6 = new ParallelComboTask();
//        TrajectorySequence toDrop_6 = drive.trajectorySequenceBuilder(toPick_6.end())
//                .lineTo(new Vector2d(param.forward1 / Math.sqrt(2), param.forward1 / Math.sqrt(2)), velFast, accelFast)  //param.forward1/Math.sqrt(2), mid 32
//                .build();
//        SplineMoveTask moveToDrop_6 = new SplineMoveTask(robotHardware.mecanumDrive, toDrop_6);
//        dropComb6.add(moveToDrop_6);
//
//        dropComb6.add(new LiftExtTutTask(robotHardware, rightwardLiftExtTut, 1));
//        seq6.add(dropComb6);
//        dropRetract6.add(seq6);
//        taskList.add(dropRetract6);
//
//        ParallelComboTask dropRetract_final = new ParallelComboTask();
//        dropRetract_final.add(new LiftArmTask(robotHardware, 800));
//        SequentialComboTask seq_final = new SequentialComboTask();
//        seq_final.add(new RobotSleep(200));
//        seq_final.add(new GrabberTask(robotHardware, true));

//        final
//        parkingRow = aprilTagSignalRecognition.getRecognitionResult();
        ParallelComboTask final_parking = new ParallelComboTask();

        //final ending parking position
        if (isRight ? parkingRow == 3 : parkingRow == 1) {
            // picking #6 cone (#5 from stack)
            ParallelComboTask comboToPick_6 = new ParallelComboTask();
            comboToPick_6.add(new DrvOpLiftExtTutTask(robotHardware, liftExtTut_6, true));
            TrajectorySequence toPick_6;
            toPick_6 = drive.trajectorySequenceBuilder(toDrop_5_Parking1_Parking2.end())
                    .lineTo(isRight ? rightStack_parking :leftStack_parking, velFast, accelFast)
                    .build();

            SplineMoveTask moveToPick6 = new SplineMoveTask(robotHardware.mecanumDrive, toPick_6);
            comboToPick_6.add(moveToPick6);

            seq6.add(comboToPick_6);
//            finalLiftExtTut_parkingNearStack.tutPos = isRight ? 1385 : -2300;
//            seq6.add(new LiftExtTutTask(robotHardware, finalLiftExtTut_parkingNearStack,1));
//            seq6.add(new GrabberTask(robotHardware, false));
//            seq6.add(new RobotSleep(150));
//            seq6.add(new LiftExtTutTask(robotHardware, pickUpConeLiftExtTut, 2));

        }else if(parkingRow == 2){ //now moving
//            finalLiftExtTut_parking2.tutPos = isRight ? -2300 : 1385;
            seq6.add(new DrvOpLiftExtTutTask(robotHardware, finalLiftExtTut_parking2, true));
        }else{ //right: parking 1 or left: parking 3
            seq6.add(new DrvOpLiftExtTutTask(robotHardware, finalLiftExtTut_parkingFarStack,true));
//            TrajectorySequence parkingNearStack;
//            parkingNearStack = drive.trajectorySequenceBuilder(toDrop_5_Parking1_Parking2.end())
//                    .lineTo(new Vector2d(49.497,21.21), velFast, accelFast)
//                    .build();
//            SplineMoveTask moveToparkingNearStack = new SplineMoveTask(robotHardware.mecanumDrive, parkingNearStack);
//            seq6.add(moveToparkingNearStack);
//            finalLiftExtTut.tutPos = isRight ? 1385 : -2300;
//            seq6.add(final_parking);
        }

        dropRetract6.add(seq6);
//        dropRetract6.add(new GrabberTask(robotHardware, false));
//        dropRetract6.add(new TurnTurretTask(robotHardware, (isRight) ? param.turretRight1FinalPos : param.turretLeft1FinalPos));
        taskList.add(dropRetract6);

//
//            SequentialComboTask finalSeq = new SequentialComboTask();
//            ParallelComboTask liftAndTurnAndPark = new ParallelComboTask();
//            liftAndTurnAndPark.add(new LiftArmTask(robotHardware,param.liftHighDrop ));//it's mid pole
//            liftAndTurnAndPark.add(new TurnTurretTask(robotHardware, (isRight) ? param.turretDropPosRightHighPole : param.turretDropPosLeftHighPole)); //450 param.turretDropPosLeft
//            liftAndTurnAndPark.add(new ExtendArmTask(robotHardware, (isRight) ? param.armLengthPostRightPickHighPole :param.armLengthPostLeftPickHighPole)); //0.05
//
//            TrajectorySequence parking;
//            if(isRight){
//                parking = drive.trajectorySequenceBuilder(toPick.end())
//                        .strafeLeft(24)   //y=54,(parkingRow==2)?24:48
//                        .build();
//            }else {
//                parking = drive.trajectorySequenceBuilder(toPick.end())
//                        .strafeRight(24)   //y=54, (parkingRow==2)?24:48
//                        .build();
//            }
//            SplineMoveTask moveToParking = new SplineMoveTask(robotHardware.mecanumDrive, parking);
//            liftAndTurnAndPark.add(moveToParking);
//            finalSeq.add(liftAndTurnAndPark);
//            finalSeq.add(new RobotSleep(270));
//            taskList.add(finalSeq);
//
//            ParallelComboTask dropRetract4 = new ParallelComboTask();
//            dropRetract4.add(new LiftArmTask(robotHardware, param.liftMidDrop));
//            SequentialComboTask seq4 = new SequentialComboTask();
//            seq4.add(new RobotSleep(150));
//            seq4.add(new GrabberTask(robotHardware, true));
//
//            ParallelComboTask finalMove = new ParallelComboTask();
//            finalMove.add(new ExtendArmTask(robotHardware, robotProfile.hardwareSpec.extensionFullInPos));
//
//            TrajectorySequence finalParking_3 = null;
//            SplineMoveTask moveToFianlPakring_3 = null;
//            if(isRight && parkingRow==1 || !isRight && parkingRow==3){
//                finalMove.add(new TurnTurretTask(robotHardware, param.turretLeft3FinalPos));//-2775 param.turretPickPosRight:param.turretPickPosLeft
//
//                if(isRight) {
//                    finalParking_3 = drive.trajectorySequenceBuilder(parking.end(), velFast)
//                            .strafeLeft(24)   //y=46
//                            .back(6)
//                            .build();
//                }else{
//                    finalParking_3 = drive.trajectorySequenceBuilder(parking.end(), velFast)
//                            .strafeRight(24)   //y=46
//                            .back(6)
//                            .build();
//                }
//
//
//            }else{ //#2
//                finalMove.add(new TurnTurretTask(robotHardware, param.turretLeft2FinalPos));//-2775 param.turretPickPosRight:param.turretPickPosLeft
//                finalParking_3 = drive.trajectorySequenceBuilder(parking.end(), velFast)
//                        .back(6)
//                        .build();
//            }
//            moveToFianlPakring_3 = new SplineMoveTask(robotHardware.mecanumDrive, finalParking_3);
//            finalMove.add(moveToFianlPakring_3);
//            finalMove.add(new GrabberTask(robotHardware, GrabberTask.GrabberState.INIT));
//            seq4.add(finalMove);
//            dropRetract4.add(seq4);
//            taskList.add(dropRetract4);
//        }
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
