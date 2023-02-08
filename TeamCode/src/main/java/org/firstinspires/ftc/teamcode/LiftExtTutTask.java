package org.firstinspires.ftc.teamcode;

public class LiftExtTutTask implements RobotControl {
        RobotHardware robotHardware;
        SoloDriverOpMode.LastLiftExtTut liftExtTut;
        long startTime;
        long modeStart;
        boolean goUp = false;
        enum Mode { STEP1, STEP2, STEP3, STEP4, DONE};
        Mode mode;

    public LiftExtTutTask(RobotHardware hardware, SoloDriverOpMode.LastLiftExtTut liftExtTut) {
        this.robotHardware = hardware;
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
            goUp = true;
        } else {    // going down, we will open after half cone height, and then retract, and then start rotate
            robotHardware.setLiftPosition(robotHardware.getRobotProfile().hardwareSpec.liftSafeRotate);
            goUp = false;
        }
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

    @Override
    public void execute() {
        if (mode==Mode.STEP1) {
            if((System.currentTimeMillis()-startTime)>200 && goUp && robotHardware.getLiftPosition()>robotHardware.getRobotProfile().hardwareSpec.liftSafeRotate){
                int error = (int) (getErrorAngle()/(2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                robotHardware.setTurretPosition(liftExtTut.tutPos + error);
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
                    robotHardware.setTurretPosition(liftExtTut.tutPos + error);
                    robotHardware.grabberOpen();
                    mode = Mode.STEP3;
                }
            }
        }
        else if (mode==Mode.STEP3) {
            if (goUp) {
                if (System.currentTimeMillis() - modeStart > 300) {
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