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
        turretMotor = robotHardware.turretMotor;

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
                activeTurretProfile = generateTurretProfile(robotHardware.getTurretPosition(), liftExtTut.tutPos);

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

    void turretStartRotate(int pos) {
        targetTurretPos = pos;
        int currPos = robotHardware.getTurretPosition();
        Logger.logFile("turretStartRotate -> curr Turret:" + currPos);
        if (Math.abs(targetTurretPos - currPos) < 300) {
            turretMode = TurretMode.PID;
            robotHardware.setTurretPosition(targetTurretPos);
            Logger.logFile("LiftExtTut Turret PID turn from:" + currPos + " to: " + targetTurretPos);
        }
        else {
            turretMode = TurretMode.POWER_1;
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            powerSign = (targetTurretPos > currPos) ? 1 : -1;
            turretMotor.setPower(powerSign * power);
            Logger.logFile("LiftExtTut Turret turn Motion Profile from:" + currPos + " to: " + targetTurretPos);
        }
    }

    int calcDistToStop(double velocity) {
        double timeToZero = velocity / robotHardware.getRobotProfile().hardwareSpec.turretDecelerate / 1000;   // second
        int dist = (int)(timeToZero * velocity / 2);
        return dist;
    }

    void turretExecute() {
        if (turretMode == TurretMode.WAIT) {
            return;
        }
        int currPos = robotHardware.getTurretPosition();
        long currTime = System.currentTimeMillis();
        double velocity = robotHardware.getTurretVelocity();
        if (turretMode == TurretMode.POWER_1) {
            int remain = powerSign * (targetTurretPos - currPos);
            int distToStop = calcDistToStop(Math.abs(velocity));
            //Logger.logFile("in turretExecute:");
            //Logger.logFile("remain:"+remain);.
            //Logger.logFile("distToStop:"+distToStop);
            if (remain <= distToStop - 20) {
                Logger.logFile("LiftExtTut Turret ramp down at " + currPos + " with velocity " + velocity);
                rampDownStart = currTime;
                turretMode = TurretMode.RAMP_DOWN;
            }
        }
        else if (turretMode == TurretMode.RAMP_DOWN) {
            int remain = powerSign * (targetTurretPos - currPos);
            //Logger.logFile("in RAMP_DOWN");
            //Logger.logFile("remain:"+remain + " powerSign:"+powerSign + " targetTurretPos:"+
            //        targetTurretPos +  " currPos:"+ currPos);
            if (remain < 50 || (currTime - rampDownStart)>1000) {
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(robotHardware.getRobotProfile().hardwareSpec.turretPower);
                turretMotor.setTargetPosition(targetTurretPos);
                turretMode = TurretMode.PID;
                Logger.logFile("LiftExtTut: Turret go PID at " + currPos);
            }
            else {
                double pwr = robotHardware.getRobotProfile().hardwareSpec.turretPower - (currTime - rampDownStart) * robotHardware.getRobotProfile().hardwareSpec.turretPowerDownMs;
                int distToStop = calcDistToStop(Math.abs(velocity));
                pwr = Math.max(0, pwr + (remain - distToStop)/(remain+distToStop) * robotHardware.getRobotProfile().hardwareSpec.turretRampDownP);
                turretMotor.setPower(pwr * powerSign);
            //    Logger.logFile("disToStop:"+distToStop);
            //    Logger.logFile("power:"+pwr);
            }
        } else if (turretMode == TurretMode.DONE) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(robotHardware.getRobotProfile().hardwareSpec.turretPower);
            turretMotor.setTargetPosition(targetTurretPos);
        }
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
//        turretExecute();
        turretExecute_RR();
        if (mode == Mode.STEP1) {
            //only for going up
            if ((System.currentTimeMillis() - startTime) > 200 && goUp && robotHardware.getLiftPosition() > robotHardware.getRobotProfile().hardwareSpec.liftSafeRotate) {
                int error = (int) (getErrorAngle() / (2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                if(!isMidPoleBeginning || isMidPoleBeginning && (System.currentTimeMillis() - startTime) > 3500){ //exclude isMidPoleBeginning and time < 3500
//                    turretStartRotate(liftExtTut.tutPos + error);//todo
                    startTurret_RR();
                    if(isMidPoleBeginning){Logger.logFile("after 3.5 second, turret start turning");}
                    modeStart = System.currentTimeMillis();
                    mode = Mode.STEP2;
                }
            }
        } else if (mode == Mode.STEP2) {
                if (goUp) {
                    if (isStackGrabLift) {
                        if (Math.abs(robotHardware.getLiftPosition() - liftExtTut.liftPos) < 30 &&
                                Math.abs(robotHardware.getExtensionPosition() - liftExtTut.extension) < 0.2) {
                            mode = Mode.DONE;
                            turretMotor.setTargetPosition(targetTurretPos);
                            turretMotor.setPower(0.3);
                            turretMode = TurretMode.DONE;
                            Logger.logFile("STEP2, End of STACK_GRAB_UP");
                        }
                    }else{
                        //check if turret is done
                        if((System.currentTimeMillis() - profileStartTime)> activeTurretProfile.duration()){
                            turretMode = TurretMode.DONE;
                            Logger.logFile("STEP 2, TurretMode.DONE! current turret:" + robotHardware.getTurretPosition());
                        }

                        //turret is done, wait until lift and extension are done -> completed the whole task
                        if (Math.abs(robotHardware.getLiftPosition() - liftExtTut.liftPos) < 30 &&
                                Math.abs(robotHardware.getExtensionPosition() - liftExtTut.extension) < 0.1 &&
                                turretMode == TurretMode.DONE) {
                            mode = Mode.DONE;
                            Logger.logFile("STEP 2-TRAVEL-up, now Mode.DONE! current turret:" + robotHardware.getTurretPosition());
                        }
                    }
                } else {  //go down
                    //!goUp occurs only after drop cone and before picking up next cone: reset the
                    //extension from armLengthPostPick to liftExtTut.extension, turret & lift started earlier already
                    if ((robotHardware.getExtensionPosition() - liftExtTut.extension) < liftExtTut.extension/2){
                        robotHardware.setExtensionPosition(liftExtTut.extension);
                        int error = (int) (getErrorAngle() / (2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                        mode = Mode.STEP3;
                    }
                }

        } else if (mode == Mode.STEP3) {
                    //this is only for !goUp
                    if (System.currentTimeMillis() - profileStartTime > activeTurretProfile.duration()) {
                        turretMode = TurretMode.DONE;
                    }else if (Math.abs(robotHardware.getLiftPosition() - liftExtTut.liftPos) < 30
                                && Math.abs(robotHardware.getExtensionPosition() - liftExtTut.extension) < 0.05) {
                            if(turretMode == TurretMode.DONE) {
                                mode = Mode.DONE;
                            }
                    }
        }
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return mode == Mode.DONE;
    }
}