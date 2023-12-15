package org.firstinspires.ftc.teamcode;

public class PixelUpTask implements RobotControl {
    RobotHardware robotHardware;
    long startTime, outTime;
    int liftPos, liftStartPos;
    enum Mode { DOWN, GRAB, UP, OUT, DONE}
    Mode mode;
    boolean isOne, isRightAprilTag;


    public PixelUpTask(RobotHardware hardware, boolean isRightAprilTag, int liftPos) {
        this.robotHardware = hardware;
        this.liftPos = this.robotHardware.getLiftPosition()+liftPos;
        isOne = true;
        this.isRightAprilTag = isRightAprilTag;
        this.liftStartPos = this.robotHardware.getLiftPosition();
        Logger.logFile("PixelUpTask liftPos:"+ this.liftPos);
        Logger.logFile("PixelUpTask liftstartPos:"+ this.liftStartPos);
    }

    public PixelUpTask(RobotHardware hardware, int liftPos, boolean isOne) {
        this.robotHardware = hardware;
        this.liftPos = liftPos;
        this.isOne = isOne;
    }

    public String toString() {
        return "PixelUpTask liftPos: " + (liftStartPos+liftPos);
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        robotHardware.stopIntake();
        robotHardware.setLiftPosition(-100);
        mode = Mode.DOWN;
    }

    @Override
    public void execute() {
        if (mode==Mode.DOWN && System.currentTimeMillis()-startTime>200) {
            Logger.logFile("in execute() get lift = " + this.robotHardware.getLiftPosition());

            robotHardware.grabberClose(isOne);
            mode = Mode.GRAB;
        }
        if (mode==Mode.GRAB && (System.currentTimeMillis()-startTime>450)) {
            mode = Mode.UP;
            robotHardware.setLiftPosition(liftPos);
            Logger.logFile("Mode.UP, lift position = "+this.robotHardware.getLiftPosition());
        }
        if (mode==Mode.UP &&
                robotHardware.getLiftPosition() > liftPos - 30) {
            Logger.logFile("get lift 1:"+robotHardware.getLiftMotors()[0].getCurrentPosition());
            Logger.logFile("get lift 2:"+robotHardware.getLiftMotors()[1].getCurrentPosition());
            robotHardware.grabberOut();
            mode = mode.OUT;
            outTime = System.currentTimeMillis();
        }
        if (mode==Mode.OUT && System.currentTimeMillis() - outTime > 200) {
            Logger.logFile("lift pos:"+robotHardware.getLiftPosition());
            if(isRightAprilTag) {
                robotHardware.grabberRight();
                Logger.logFile("pixelUpTask, grabber turns right");
            }else {
                robotHardware.grabberLeft();
                Logger.logFile("pixelUpTask, grabber turns left");
            }
            mode = Mode.DONE;
        }
    }
    @Override
    public void cleanUp() {
        robotHardware.setLiftPosition(liftPos);
    }

    @Override
    public boolean isDone() {
        return mode==mode.DONE;
    }
}
