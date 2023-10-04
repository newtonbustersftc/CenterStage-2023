package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftExtTutTask implements RobotControl {
        RobotHardware robotHardware;
        SoloDriverOpMode.LastLiftExtTut liftExtTut;
        long startTime;
        long modeStart;
        boolean goUp = false;
        enum Mode { STEP1, STEP2, STEP3, STEP4, DONE};
        Mode mode;

        //Autonomous
        boolean isAutonomous = false;
        double autonomousStartTime;
        boolean isMidPole = false;
        boolean isMidPoleBeginning = false;
        enum Autonmous_Action{TRAVEL,STACK_GRAB_UP}
        Autonmous_Action autonmous_action;

        // motion profile turret parameters
        enum TurretMode { WAIT, POWER_1, RAMP_DOWN, PID, DONE };
        TurretMode turretMode;
        DcMotorEx turretMotor;
        int targetTurretPos;
        int powerSign = 1;
        double power;
        long rampDownStart;

    public LiftExtTutTask(RobotHardware hardware, SoloDriverOpMode.LastLiftExtTut liftExtTut) {
        this.robotHardware = hardware;
        this.liftExtTut = liftExtTut;
    }

    //Autonomous=> operationNum :
    // 0=mid pole beginning
    // 1=wheel movement to all locations,
    // 2=at stack, grab & lift
    // 3=mid pole, not beginning
    public LiftExtTutTask(RobotHardware hardware, SoloDriverOpMode.LastLiftExtTut liftExtTut, double operationNum) {
        this.robotHardware = hardware;
        this.liftExtTut = liftExtTut;

        if(operationNum == 0) {
            autonmous_action = Autonmous_Action.TRAVEL;
            isMidPole = true;
            isMidPoleBeginning = true;
            Logger.logFile("am I here in 0?");
        }else if(operationNum == 1){
            autonmous_action = Autonmous_Action.TRAVEL;
        }else if(operationNum == 2){
            autonmous_action = Autonmous_Action.STACK_GRAB_UP;
        }
//        else if(operationNum == 3){
//            autonmous_action = Autonmous_Action.TRAVEL;
//            isMidPole = true;
//            Logger.logFile("am I here in 3?");
//        }
        this.isAutonomous = true;
        Logger.logFile("1");
    }

    public String toString() {
        return "LiftExtTut lift:" + liftExtTut.liftPos +
        "LiftExtTut tutPos: " + liftExtTut.tutPos +
        "LiftExtTut ext: " + liftExtTut.extension ;
    }

    @Override
    public void prepare() {
        mode = Mode.STEP1;
        if(isAutonomous) {
            if(autonmous_action == Autonmous_Action.TRAVEL){
                if(robotHardware.getLiftPosition() <= liftExtTut.liftPos){
                    Logger.logFile("2, prepare, goUp, delay set Ext to liftExtTut.extension ");
                    robotHardware.setExtensionPosition(liftExtTut.extension);
                    Logger.logFile("set extension");

                    goUp = true;
                    mode = LiftExtTutTask.Mode.STEP1;  //motion profile for up
                }else { //go down
//                    if(!isMidPole) {
                        Logger.logFile("2, prepare, goDown, set Ext to init extension - 0.29");
                        robotHardware.setExtensionPosition(robotHardware.getRobotProfile().autonParam.armLengthPostPick);
//                    }
                    goUp = false;
                    mode = LiftExtTutTask.Mode.STEP2;
                }

                //lift up later for midpole beginning, otherwise outside of the wall
                robotHardware.setLiftPositionUnsafe(liftExtTut.liftPos, 0.9);
            }else if(autonmous_action == Autonmous_Action.STACK_GRAB_UP){
                robotHardware.setLiftPosition(liftExtTut.liftPos);
                robotHardware.setExtensionPosition(liftExtTut.extension);
                autonomousStartTime = System.currentTimeMillis();
                goUp = true;
                mode = LiftExtTutTask.Mode.STEP1;  //motion profile for up  **test!!!!!
            }
        }else { //non-autonomous
            if (robotHardware.getTargetLiftPosition() < liftExtTut.liftPos) { //up up and away
                robotHardware.setExtensionPosition(robotHardware.getRobotProfile().hardwareSpec.extensionDriverMin);
                robotHardware.setLiftPositionUnsafe(liftExtTut.liftPos, 0.9);
                turretMode = TurretMode.WAIT;
                goUp = true;
            } else {    // going down, we will open after half cone height, and then retract, and then start rotate
                robotHardware.setLiftPosition(robotHardware.getRobotProfile().hardwareSpec.liftSafeRotate);
                turretMode = TurretMode.WAIT;
                goUp = false;
            }
        }
        power = robotHardware.getRobotProfile().hardwareSpec.turretPower;
        startTime = System.currentTimeMillis();
        //Logger.logFile("in prepare, turret is:"+ robotHardware.getTurretPosition());
        //Logger.logFile("Prepare, tutPos:" + liftExtTut.tutPos + " go up:" + goUp);
    }

    double getErrorAngle() {
//        Logger.logFile("current gyro:"+ robotHardware.getGyroHeading());
//        double errAngle = robotHardware.getGyroHeading() - liftExtTut.robHead;
//        if (errAngle>Math.PI) {
//            errAngle = errAngle - Math.PI*2;
//        }
//        else if (errAngle < -Math.PI) {
//            errAngle = errAngle + Math.PI*2;
//        }
//        Logger.logFile("errAngle:"+errAngle);
//        return errAngle;
        return 0;
    }

    int calcDistToStop(double velocity) {
        double timeToZero = velocity / robotHardware.getRobotProfile().hardwareSpec.turretDecelerate / 1000;   // second
        int dist = (int)(timeToZero * velocity / 2);
        return dist;
    }

    @Override
    public void execute() {

    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return mode == Mode.DONE;
    }
}