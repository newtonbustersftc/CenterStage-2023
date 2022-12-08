package org.firstinspires.ftc.teamcode;

public class LiftArmTask implements RobotControl {
        RobotHardware robotHardware;
        int currPos;
        int liftPos;
        long startTime;
        boolean isUnsafe;

    public LiftArmTask(RobotHardware hardware, int liftPos) {
        this.robotHardware = hardware;
        this.liftPos = liftPos;
    }

    public LiftArmTask(RobotHardware hardware, int liftPos, boolean isUnsafe){
        this.robotHardware = hardware;
        this.liftPos = liftPos;
        this.isUnsafe = isUnsafe;
    }

    public String toString() {
        return "Lift arm to " + liftPos;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        currPos = robotHardware.getLiftPosition();
        if(isUnsafe){
            robotHardware.setLiftPositionUnsafe(liftPos,0.5);
        }else {
            robotHardware.setLiftPosition(liftPos);
        }
        Logger.logFile("LiftBucketTask: lift pos = " + liftPos);
    }

    @Override
    public void execute() {
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return (System.currentTimeMillis()-startTime)>100 && !robotHardware.isLiftMoving();
    }
}