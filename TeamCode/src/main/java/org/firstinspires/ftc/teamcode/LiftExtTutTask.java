package org.firstinspires.ftc.teamcode;

public class LiftExtTutTask implements RobotControl {
        RobotHardware robotHardware;
        SoloDriverOpMode.LastLiftExtTut liftExtTut;
        long startTime;
        long moveExtStart;

    public LiftExtTutTask(RobotHardware hardware, SoloDriverOpMode.LastLiftExtTut liftExtTut) {
        this.robotHardware = hardware;
        this.liftExtTut = liftExtTut;
    }

    public String toString() {
        return "LiftExtTut lift:" + liftExtTut.liftPos;
    }

    @Override
    public void prepare() {
        moveExtStart = -1;  // 3 second from now
        robotHardware.setExtensionPosition(robotHardware.getRobotProfile().hardwareSpec.extensionDriverMin);
        robotHardware.setLiftPositionUnsafe(liftExtTut.liftPos, 0.6);
        robotHardware.setTurretPosition(liftExtTut.tutPos);
        startTime = System.currentTimeMillis();
        Logger.logFile("Prepare LiftExtTut lift:" + liftExtTut.liftPos);
    }

    @Override
    public void execute() {
        if (moveExtStart==-1) {
            if ((System.currentTimeMillis()-startTime)>200 && !robotHardware.isLiftMoving() && !robotHardware.isTurretTurning() ||
                (System.currentTimeMillis()-startTime)>2000) {
                Logger.logFile("LiftExtTut move extension");
                robotHardware.setExtensionPosition(liftExtTut.extension);
                moveExtStart = System.currentTimeMillis();
            }
        }
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return moveExtStart!=-1 && (System.currentTimeMillis()-moveExtStart)>300;   // assume 0.3 second to move extension
    }
}