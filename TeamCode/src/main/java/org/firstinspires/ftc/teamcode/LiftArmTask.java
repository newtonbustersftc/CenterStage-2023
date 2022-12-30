package org.firstinspires.ftc.teamcode;

public class LiftArmTask implements RobotControl {
        RobotHardware robotHardware;
        int currPos;
        int liftPos;
        long startTime;
        boolean isUnsafe;
        boolean isAddition;

    public LiftArmTask(RobotHardware hardware, int liftPos) {
        this.robotHardware = hardware;
        this.liftPos = liftPos;
        this.isAddition = false;
        this.isUnsafe = false;
    }

    public LiftArmTask(RobotHardware hardware, int liftPos, boolean isUnsafe){
        this.robotHardware = hardware;
        this.liftPos = liftPos;
        this.isUnsafe = isUnsafe;
        isAddition = false;
    }

    public LiftArmTask(RobotHardware hardware, int deltaPos, boolean isUnsafe, boolean isAddition) {
        this.robotHardware = hardware;
        this.liftPos = deltaPos;
        this.isUnsafe = isUnsafe;
        this.isAddition = isAddition;
    }

    public String toString() {
        return "Lift arm to " + liftPos;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        currPos = robotHardware.getLiftPosition();
        int newPos = isAddition ? currPos + liftPos : liftPos;
        if(isUnsafe){
            robotHardware.setLiftPositionUnsafe(newPos,0.5);
        }else {
            robotHardware.setLiftPosition(newPos);
        }
        Logger.logFile("LiftBucketTask: lift pos = " + newPos);
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