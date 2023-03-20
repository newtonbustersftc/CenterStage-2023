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
        turretMotor = robotHardware.turretMotor;
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

        turretMotor = robotHardware.turretMotor;
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
                if (!isMidPoleBeginning){
                    robotHardware.setTurretPosition(liftExtTut.tutPos);
                }else{
                    turretMode = LiftExtTutTask.TurretMode.WAIT;
                    Logger.logFile("no turret start, TurretMode.WAIT");
                }
                //Logger.logFile("prepare, getTurretPosition="+robotHardware.getTurretPosition());
                //Logger.logFile("prepare, liftWxtTut.liftPos="+liftExtTut.liftPos);
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

    @Override
    public void execute() {
        turretExecute();
        if (mode == Mode.STEP1) {
            //only for going up
            if ((System.currentTimeMillis() - startTime) > 200 && goUp && robotHardware.getLiftPosition() > robotHardware.getRobotProfile().hardwareSpec.liftSafeRotate) {
                int error = (int) (getErrorAngle() / (2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                if(isMidPoleBeginning && ((System.currentTimeMillis() - startTime) > 3500)){
                    turretStartRotate(liftExtTut.tutPos + error);
                    Logger.logFile("after 3.5 second, turret start turning");
                }else{
                    turretStartRotate(liftExtTut.tutPos + error);
                }
                modeStart = System.currentTimeMillis();
                mode = Mode.STEP2;
                Logger.logFile("STEP1, I am here because I am going up ");
            }
        } else if (mode == Mode.STEP2) {
            if (isAutonomous) {
                if (goUp) {
                    //Logger.logFile("I am in STEP 2, going up");
                    if (autonmous_action == Autonmous_Action.TRAVEL) {
                        if (Math.abs(robotHardware.getLiftPosition() - liftExtTut.liftPos) < 30 &&
                                Math.abs(robotHardware.getExtensionPosition() - liftExtTut.extension) < 0.1 &&
                                turretMode == TurretMode.DONE) {
                            mode = Mode.DONE;
                            Logger.logFile("STEP 2-TRAVEL, now Mode.DONE! current turret:" + robotHardware.getTurretPosition());
                        }
                        if (powerSign*(targetTurretPos - robotHardware.getTurretPosition()) < 20) {
                            turretMode = TurretMode.DONE;
                            //Logger.logFile("4. STEP 2, TurretMode.DONE! current turret:" + robotHardware.getTurretPosition());
                        }
                    } else if (autonmous_action == Autonmous_Action.STACK_GRAB_UP) {
                        //Logger.logFile("I am in STEP 2, STACK_GRAB_UP");
                        if (Math.abs(robotHardware.getLiftPosition() - liftExtTut.liftPos) < 30 &&
                                Math.abs(robotHardware.getExtensionPosition() - liftExtTut.extension) < 0.2) {
                            mode = LiftExtTutTask.Mode.DONE;
                            turretMotor.setTargetPosition(targetTurretPos);
                            turretMotor.setPower(0.3);
                            turretMode = LiftExtTutTask.TurretMode.DONE;
                            Logger.logFile("4. STEP2, End of STACK_GRAB_UP");
                        }
                    }
                    //Logger.logFile("I am in STEP 2, not going anywhere");
                    if (mode != Mode.DONE && turretMode != TurretMode.DONE) {
                        //Logger.logFile("4. STEP 2, Not DONE! current turret:" + robotHardware.getTurretPosition());
                    }
                } else { //autonomous go down
                    if (autonmous_action == Autonmous_Action.TRAVEL) {
                        //Logger.logFile("4. goDown, in STEP2, current extension = " + robotHardware.getExtensionPosition());
                        robotHardware.setExtensionPosition(liftExtTut.extension);
                        int error = (int) (getErrorAngle() / (2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
//                        robotHardware.setTurretPosition(liftExtTut.tutPos + error);
                        turretStartRotate(liftExtTut.tutPos + error);

                        //Logger.logFile("4. goDown, in STEP2, error = " + error);
                        //Logger.logFile("4. goDown, in STEP2, tutPos = liftExtTut.tutPos=" + liftExtTut.tutPos);
                        //Logger.logFile("4. goDown, in STEP2, turret = " + (liftExtTut.tutPos + error));
                        //Logger.logFile("4. goDown, Travel -> STEP3");
                        mode = LiftExtTutTask.Mode.STEP3;
                    }
                }
            }else { //not autonomous
                    //Logger.logFile("Autonomous should not be here");
                    if (goUp) {
                        if (System.currentTimeMillis() - modeStart > 200 && !robotHardware.isLiftMoving()) {
                            mode = Mode.STEP3;
                            modeStart = System.currentTimeMillis();
                            robotHardware.setExtensionPosition(liftExtTut.extension);
                        }
                    } else {
                        if (System.currentTimeMillis() - modeStart > 400) {
                            int error = (int) (getErrorAngle() / (2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                            //robotHardware.setTurretPosition(liftExtTut.tutPos + error);
                            turretStartRotate(liftExtTut.tutPos + error);
                            robotHardware.grabberOpen();
                            mode = Mode.STEP3;
                        }
                    }
            }
        } else if (mode == Mode.STEP3) {
                if (isAutonomous) {
                    if (Math.abs(robotHardware.getLiftPosition() - liftExtTut.liftPos) < 30
                            && Math.abs(robotHardware.getExtensionPosition() - liftExtTut.extension) < 0.05
                            && powerSign*( targetTurretPos -robotHardware.getTurretPosition()) < 30) {
                        //Logger.logFile("5, STEP3, end of isAutonomous ->TRAVEL:" + robotHardware.getTurretPosition());
                        //Logger.logFile("5. STEP3, liftPos:" + robotHardware.getLiftPosition());
                        //Logger.logFile("5. STEP3, extension:" + robotHardware.getExtensionPosition());
                        //Logger.logFile("5, STEP3, cur turret: " + robotHardware.getTurretPosition());
                        //Logger.logFile("5, STEP3, targetTurret: " + targetTurretPos);
                        turretMotor.setTargetPosition(targetTurretPos);
                        turretMotor.setPower(0.3);
                        turretMode = TurretMode.DONE;

                        mode = Mode.DONE;
                        //Logger.logFile("am I here?");
                        //Logger.logFile("5. STEP3, end of TRAVEL up or down, end of Autonomous -- DONE");
                    } else if (System.currentTimeMillis() - modeStart > 300 && turretMode == LiftExtTutTask.TurretMode.PID) {
                        mode = LiftExtTutTask.Mode.DONE;
                        turretMode = LiftExtTutTask.TurretMode.DONE;
                        //Logger.logFile("5. STEP3, current turret:" + robotHardware.getTurretPosition());
                        //Logger.logFile("5. STEP3, current turret:" + robotHardware.getTurretPosition());
                        //Logger.logFile("5. STEP3, System.currentTimeMillis() - modeStart > 300 && turretMode==TurretMode.PID   -- DONE");
                    }
                } else {//non autonomous
                    //Logger.logFile("Autonomous should not be here");
                    if (goUp) {
                        if (System.currentTimeMillis() - modeStart > 300 && turretMode == TurretMode.PID) {
                            mode = Mode.DONE;
                        }
                    } else if (Math.abs(robotHardware.getTurretPosition() - liftExtTut.tutPos) < 200) {
                        robotHardware.setExtensionPosition((robotHardware.getRobotProfile().hardwareSpec.extensionDriverMin + liftExtTut.extension) / 2);
                        robotHardware.setLiftPosition(liftExtTut.liftPos);
                        mode = Mode.STEP4;
                    }
                }
        } else if (mode == Mode.STEP4) {
                Logger.logFile("turret:" + robotHardware.getTurretPosition());
                if (isAutonomous) {
                    //Logger.logFile("Autonomous should not be here");
                } else { //none Autonomous
                    //Logger.logFile("Autonomous should not be here");

                    if (Math.abs(robotHardware.getLiftPosition() - liftExtTut.liftPos) < 30) {
                        robotHardware.setExtensionPosition(liftExtTut.extension);
                        turretMotor.setTargetPosition(targetTurretPos);
                        turretMotor.setPower(robotHardware.getRobotProfile().hardwareSpec.turretPower);
                        turretMode = TurretMode.DONE;
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