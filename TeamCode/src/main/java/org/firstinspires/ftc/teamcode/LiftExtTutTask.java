package org.firstinspires.ftc.teamcode;

public class LiftExtTutTask implements RobotControl {
        RobotHardware robotHardware;
        SoloDriverOpMode.LastLiftExtTut liftExtTut;
        long startTime;
        long modeStart;
        boolean goUp = false;
        enum Mode { STEP1, STEP2, STEP3, DONE};
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
            robotHardware.setLiftPositionUnsafe(liftExtTut.liftPos, 0.6);
            goUp = true;
        } else {    // going down
            int error = (int) ((robotHardware.getGyroHeading() - liftExtTut.robHead)/(2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
            robotHardware.setExtensionPosition(robotHardware.getRobotProfile().hardwareSpec.extensionDriverMin);
            robotHardware.setTurretPosition(liftExtTut.tutPos + error);
            Logger.logFile("LiftExtTut move turret to " + liftExtTut.tutPos);
            goUp = false;
        }
        startTime = System.currentTimeMillis();
        Logger.logFile("Prepare LiftExtTut lift:" + liftExtTut.liftPos + " go up:" + goUp);
    }

    @Override
    public void execute() {
        if (mode==Mode.STEP1) {
            if((System.currentTimeMillis()-startTime)>200 && goUp && robotHardware.getLiftPosition()>robotHardware.getRobotProfile().hardwareSpec.liftSafeRotate){
                int error = (int) ((robotHardware.getGyroHeading() - liftExtTut.robHead)/(2 * Math.PI) * robotHardware.getRobotProfile().hardwareSpec.turret360);
                robotHardware.setTurretPosition(liftExtTut.tutPos + error);

                Logger.logFile("LiftExtTut move turret to " + liftExtTut.tutPos);  //while still lifting
                modeStart = System.currentTimeMillis();
                mode = Mode.STEP2;
            } else if((System.currentTimeMillis()-startTime)>200 && !robotHardware.isTurretTurning() && !goUp){
                robotHardware.setLiftPositionUnsafe(liftExtTut.liftPos, 0.6);
                robotHardware.setExtensionPosition(liftExtTut.extension);
                Logger.logFile("LiftExtTut move lift to " + liftExtTut.liftPos);
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
                if (System.currentTimeMillis() - modeStart>300 && !robotHardware.isLiftMoving()) {
                    mode = Mode.DONE;
                }
            }
        }
        else if (mode==Mode.STEP3) {
            if (System.currentTimeMillis() - modeStart>300) {
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