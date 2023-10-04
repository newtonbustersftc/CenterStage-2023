package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DrvOpLiftExtTutTask implements RobotControl {
        RobotHardware robotHardware;
        SoloDriverOpMode.LastLiftExtTut liftExtTut;
        long startTime;
        long modeStart;
        boolean goUp = false;
        enum Mode { STEP1, STEP2, STEP3, STEP4, DONE};
        Mode mode;
        // motion profile turret parameters
        enum TurretMode { WAIT, POWER_1, RAMP_DOWN, PID };
        TurretMode turretMode;
        DcMotorEx turretMotor;
        int targetTurretPos;
        int powerSign = 1;
        double power;
        long rampDownStart;
        boolean noErrAngle = false;

    public DrvOpLiftExtTutTask(RobotHardware hardware, SoloDriverOpMode.LastLiftExtTut liftExtTut) {
        this.robotHardware = hardware;
        this.liftExtTut = liftExtTut;
    }

    public DrvOpLiftExtTutTask(RobotHardware hardware, SoloDriverOpMode.LastLiftExtTut liftExtTut, boolean noErrAngle) {
        this.robotHardware = hardware;
        this.liftExtTut = liftExtTut;
        this.noErrAngle = noErrAngle;
    }

    public String toString() {
        return "LiftExtTut (" + liftExtTut + ")";
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
        Logger.logFile("Prepare LiftExtTut lift to (" + liftExtTut + ") go up:" + goUp);
    }

    double getErrorAngle() {
        if (noErrAngle) {
            return 0;
        }
        double errAngle = robotHardware.getGyroHeading() - liftExtTut.robHead;
        if (errAngle>Math.PI) {
            errAngle = errAngle - Math.PI*2;
        }
        else if (errAngle < -Math.PI) {
            errAngle = errAngle + Math.PI*2;
        }
        return errAngle;
    }

    int calcDistToStop(double velocity) {
        double timeToZero = velocity / robotHardware.getRobotProfile().hardwareSpec.turretDecelerate / 1000;   // second
        int dist = (int)(timeToZero * velocity / 2);
        return dist;
    }
    @Override
    public void execute() {
        if (mode== Mode.STEP1) {
            if((System.currentTimeMillis()-startTime)>200 && goUp) {
                    //&& robotHardware.getLiftPosition()>robotHardware.getRobotProfile().hardwareSpec.liftSafeRotate){
                int error = (int) (getErrorAngle()/(2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                //robotHardware.setTurretPosition(liftExtTut.tutPos + error);
                Logger.logFile("LiftExtTut move turret to " + liftExtTut.tutPos);  //while still lifting
                modeStart = System.currentTimeMillis();
                mode = Mode.STEP2;
            } else if((System.currentTimeMillis()-startTime)>50 && !goUp){
                robotHardware.grabberInit();    // after start go down, open grabber full first and retract
                robotHardware.setExtensionPosition(robotHardware.getRobotProfile().hardwareSpec.extensionDriverMin);
                Logger.logFile("LiftExtTut (" + liftExtTut + ") opening grabber while going down");  //while still lifting
                modeStart = System.currentTimeMillis();
                mode = Mode.STEP2;
            }
        }
        else if (mode== Mode.STEP2) {
            if (goUp) {
                if ((System.currentTimeMillis() - modeStart)>200 && !robotHardware.isLiftMoving() && turretMode==TurretMode.PID){
                    mode = Mode.STEP3;
                    modeStart = System.currentTimeMillis();
                    robotHardware.setExtensionPosition(liftExtTut.extension);
                }
            }
            else {
                if ((System.currentTimeMillis() - modeStart)>100) {
                    int error = (int) (getErrorAngle()/(2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                    robotHardware.grabberOpen();
                    modeStart = System.currentTimeMillis();
                    mode = Mode.STEP3;
                    Logger.logFile("LiftExtTut (" + liftExtTut + ") going STEP3");
                }
            }
        }
        else if (mode== Mode.STEP3) {
            if (goUp) {
                if (System.currentTimeMillis() - modeStart > 200 && turretMode==TurretMode.PID) {
                    mode = Mode.DONE;
                }
            }

        }
        else if (mode== Mode.STEP4) {
            if (Math.abs(robotHardware.getLiftPosition() - liftExtTut.liftPos) < 30 && turretMode==TurretMode.PID) {
                robotHardware.setExtensionPosition(liftExtTut.extension);
                mode = Mode.DONE;
                Logger.logFile("LiftExtTut (" + liftExtTut + ") DONE");
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