package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import android.util.Log;

import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftExtTutTask_MotionProfile implements RobotControl {
        RobotHardware robotHardware;
        SoloDriverOpMode.LastLiftExtTut liftExtTut;
        long startTime;
        long modeStart;
        boolean goUp = false;
        enum Mode { STEP1, STEP2, STEP3, DONE};
        Mode mode;

        //Autonomous
        double autonomousStartTime;
        boolean isMidPole = false;
        boolean isMidPoleBeginning = false;
        boolean isStackGrabLift = false;
        enum Autonmous_Action{TRAVEL}
        Autonmous_Action autonmous_action;

        // motion profile turret parameters
        enum TurretMode { WAIT, POWER_1, RAMP_DOWN, PID, DONE };
        TurretMode turretMode;
        DcMotorEx turretMotor;
        int targetTurretPos;
        int powerSign = 1;
        double power;
        long rampDownStart;

        //RR motion profile turret:
        MotionProfile activeTurretProfile;
        long turretProfileStart;
        MotionState motionState;
        double targetTurretPower = 0;
        long profileStartTime;

    //Autonomous=> operationNum :
    // 0=mid pole beginning
    // 1=wheel movement to all locations,
    // 2=at stack, grab & lift
    public LiftExtTutTask_MotionProfile(RobotHardware hardware, SoloDriverOpMode.LastLiftExtTut liftExtTut, double operationNum) {
        this.robotHardware = hardware;
        this.liftExtTut = liftExtTut;

        if(operationNum == 0) {
            isMidPole = true;
            isMidPoleBeginning = true;
        }else if(operationNum == 2){  //skip operationNum==1, do nothing to 1
            isStackGrabLift = true;
        }
    }

    public String toString() {
        return "LiftExtTut lift:" + liftExtTut.liftPos +
        "LiftExtTut tutPos: " + liftExtTut.tutPos +
        "LiftExtTut ext: " + liftExtTut.extension ;
    }

    @Override
    public void prepare() {
        mode = Mode.STEP1;
            if(isStackGrabLift){
                robotHardware.setLiftPosition(liftExtTut.liftPos);
                robotHardware.setExtensionPosition(liftExtTut.extension);
                autonomousStartTime = System.currentTimeMillis();
                goUp = true;
                mode = LiftExtTutTask_MotionProfile.Mode.STEP1;  //motion profile for up  **test!!!!!
            }else{
                if(robotHardware.getLiftPosition() <= liftExtTut.liftPos){
                    Logger.logFile("2, prepare, goUp, delay set Ext to liftExtTut.extension ");
                    robotHardware.setExtensionPosition(liftExtTut.extension);

                    goUp = true;
                    mode = LiftExtTutTask_MotionProfile.Mode.STEP1;  //motion profile for up
                }else { //go down
                    Logger.logFile("2, prepare, goDown, start turning turret and set Ext to armLengthPostPick - 0.13");
                    startTurret_RR();
                    robotHardware.setExtensionPosition(robotHardware.getRobotProfile().autonParam.armLengthPostPick);
                    goUp = false;
                    mode = LiftExtTutTask_MotionProfile.Mode.STEP2; //for simplicity, STEP1 is only handling goUp
                }

                robotHardware.setLiftPositionUnsafe(liftExtTut.liftPos, 0.9);

                if (!isMidPoleBeginning){
//                    robotHardware.setTurretPosition(liftExtTut.tutPos);
                    motionState = activeTurretProfile.get(turretProfileStart);
                }else{
                    turretMode = LiftExtTutTask_MotionProfile.TurretMode.WAIT;
                    Logger.logFile("no turret start, TurretMode.WAIT");
                }
            }

        power = robotHardware.getRobotProfile().hardwareSpec.turretPower;
        startTime = System.currentTimeMillis();
    }

    private static MotionProfile generateTurretProfile(double startTurretPos, double goalTurretPos) {
        MotionState start = new MotionState(startTurretPos, 0, 0, 0);
        MotionState goal = new MotionState( goalTurretPos, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
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

    private void turretExecute_RR(){
        if(turretMotor.getPower() != 0) {
            Logger.logFile("I am not here unless I already start the turret");
            MotionState motionState = activeTurretProfile.get(System.currentTimeMillis() - profileStartTime);
            targetTurretPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);
            turretMotor.setPower(targetTurretPower);
        }
    }

    private void startTurret_RR() {
        if (turretMotor.getPower() == 0.0) { //starting the turret only when the power is 0
            Logger.logFile("I am not here unless I start the turret");
            profileStartTime = System.currentTimeMillis();
            int sign = (liftExtTut.tutPos - turretMotor.getCurrentPosition() > 0 ? 1 : -1);
            turretMotor.setPower(sign * 0.01); //just want to make turret motor power not equal to 0 to start the motionProfile
            turretExecute_RR();
        }else{
            Logger.logFile("I am not suppose to be here in starting turret");
        }
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