package org.firstinspires.ftc.teamcode;

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

    public String toString() {
        return "LiftExtTut lift:" + liftExtTut.liftPos;
    }

    @Override
    public void prepare() {
        mode = Mode.STEP1;
        if(robotHardware.getTargetLiftPosition() < liftExtTut.liftPos){ //up up and away
            robotHardware.setExtensionPosition(robotHardware.getRobotProfile().hardwareSpec.extensionDriverMin);
            robotHardware.setLiftPositionUnsafe(liftExtTut.liftPos, 0.9);
            turretMode = TurretMode.WAIT;
            goUp = true;
        } else {    // going down, we will open after half cone height, and then retract, and then start rotate
            robotHardware.setLiftPosition(robotHardware.getRobotProfile().hardwareSpec.liftSafeRotate);
            turretMode = TurretMode.WAIT;
            goUp = false;
        }
        power = robotHardware.getRobotProfile().hardwareSpec.turretPower;
        startTime = System.currentTimeMillis();
        Logger.logFile("Prepare LiftExtTut lift:" + liftExtTut.liftPos + " go up:" + goUp);
    }

    double getErrorAngle() {
        double errAngle = robotHardware.getGyroHeading() - liftExtTut.robHead;
        if (errAngle>Math.PI) {
            errAngle = errAngle - Math.PI*2;
        }
        else if (errAngle < -Math.PI) {
            errAngle = errAngle + Math.PI*2;
        }
        return errAngle;
    }

    void turretStartRotate(int pos) {
        targetTurretPos = pos;
        int currPos = robotHardware.getTurretPosition();
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
        if (turretMode == TurretMode.POWER_1) {
            int remain = Math.abs(targetTurretPos - currPos);
            double velocity = robotHardware.getTurretVelocity();
            int distToStop = calcDistToStop(Math.abs(velocity));
            if (remain <= distToStop - 20) {
                Logger.logFile("LiftExtTut Turret ramp down at " + currPos + " with velocity " + velocity);
                rampDownStart = System.currentTimeMillis();
                turretMode = TurretMode.RAMP_DOWN;
            }
        }
        else if (turretMode == TurretMode.RAMP_DOWN) {
            int remain = powerSign * (targetTurretPos - currPos);
            if (remain < 100 || (System.currentTimeMillis() - rampDownStart)>1000) {
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(robotHardware.getRobotProfile().hardwareSpec.turretPower);
                turretMotor.setTargetPosition(targetTurretPos);
                turretMode = TurretMode.PID;
                Logger.logFile("LiftExtTut Turret go PID at " + currPos);
            }
            else {
                double pwr = Math.max(0.05, power - (System.currentTimeMillis() - rampDownStart) * robotHardware.getRobotProfile().hardwareSpec.turretPowerDownMs);
                double velocity = robotHardware.getTurretVelocity();
                int distToStop = calcDistToStop(Math.abs(velocity));
                pwr = pwr + (remain - distToStop)/remain * robotHardware.getRobotProfile().hardwareSpec.turretRampDownP;
                turretMotor.setPower(pwr * powerSign);
            }
        }
        else if (turretMode == TurretMode.DONE) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(robotHardware.getRobotProfile().hardwareSpec.turretPower);
            turretMotor.setTargetPosition(targetTurretPos);
        }
    }


    @Override
    public void execute() {
        turretExecute();
        if (mode==Mode.STEP1) {
            if((System.currentTimeMillis()-startTime)>200 && goUp && robotHardware.getLiftPosition()>robotHardware.getRobotProfile().hardwareSpec.liftSafeRotate){
                int error = (int) (getErrorAngle()/(2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                //robotHardware.setTurretPosition(liftExtTut.tutPos + error);
                turretStartRotate(liftExtTut.tutPos + error);
                Logger.logFile("LiftExtTut move turret to " + liftExtTut.tutPos);  //while still lifting
                modeStart = System.currentTimeMillis();
                mode = Mode.STEP2;
            } else if((System.currentTimeMillis()-startTime)>50 && !goUp){
                robotHardware.grabberInit();    // after start go down, open grabber full first and retract
                robotHardware.setExtensionPosition(robotHardware.getRobotProfile().hardwareSpec.extensionDriverMin);
                modeStart = System.currentTimeMillis();
                mode = Mode.STEP2;
            }
        }
        else if (mode==Mode.STEP2) {
            if (goUp) {
                if (System.currentTimeMillis() - modeStart>200 && !robotHardware.isLiftMoving()){
                    mode = Mode.STEP3;
                    modeStart = System.currentTimeMillis();
                    robotHardware.setExtensionPosition(liftExtTut.extension);
                }
            }
            else {
                if (System.currentTimeMillis() - modeStart>400) {
                    int error = (int) (getErrorAngle()/(2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                    //robotHardware.setTurretPosition(liftExtTut.tutPos + error);
                    turretStartRotate(liftExtTut.tutPos + error);
                    robotHardware.grabberOpen();
                    mode = Mode.STEP3;
                }
            }
        }
        else if (mode==Mode.STEP3) {
            if (goUp) {
                if (System.currentTimeMillis() - modeStart > 300 && turretMode==TurretMode.PID) {
                    mode = Mode.DONE;
                }
            }
            else if (Math.abs(robotHardware.getTurretPosition()- liftExtTut.tutPos)<200) {
                robotHardware.setExtensionPosition((robotHardware.getRobotProfile().hardwareSpec.extensionDriverMin + liftExtTut.extension)/2);
                robotHardware.setLiftPosition(liftExtTut.liftPos);
                mode = Mode.STEP4;
            }
        }
        else if (mode==Mode.STEP4) {
            if (Math.abs(robotHardware.getLiftPosition() - liftExtTut.liftPos) < 30) {
                robotHardware.setExtensionPosition(liftExtTut.extension);
                turretMotor.setTargetPosition(targetTurretPos);
                turretMotor.setPower(robotHardware.getRobotProfile().hardwareSpec.turretPower);
                turretMode = TurretMode.DONE;
                mode = Mode.DONE;
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