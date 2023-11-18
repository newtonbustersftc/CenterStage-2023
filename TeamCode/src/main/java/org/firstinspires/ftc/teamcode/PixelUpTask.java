package org.firstinspires.ftc.teamcode;

public class PixelUpTask implements RobotControl {
    RobotHardware robotHardware;
    long startTime, outTime;
    int liftPos;
    enum Mode { DOWN, GRAB, UP, OUT}
    Mode mode;
    boolean isOne;

    public PixelUpTask(RobotHardware hardware, int liftPos) {
        this.robotHardware = hardware;
        this.liftPos = liftPos;
        isOne = true;
    }

    public PixelUpTask(RobotHardware hardware, int liftPos, boolean isOne) {
        this.robotHardware = hardware;
        this.liftPos = liftPos;
        this.isOne = isOne;
    }

    public String toString() {
        return "PixelUpTask liftPos: " + liftPos;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        robotHardware.setLiftPosition(-100);
        mode = Mode.DOWN;
    }

    @Override
    public void execute() {
        if (mode==Mode.DOWN && System.currentTimeMillis()-startTime>100) {
            robotHardware.grabberClose(isOne);
            mode = Mode.GRAB;
        }
        if (mode==Mode.GRAB && (System.currentTimeMillis()-startTime>350)) {
            mode = Mode.UP;
            robotHardware.setLiftPosition(liftPos);
        }
        if (mode==Mode.UP &&
                robotHardware.getLiftPosition() > robotHardware.getRobotProfile().hardwareSpec.liftOutMin) {
            robotHardware.grabberOut();
            mode = mode.OUT;
            outTime = System.currentTimeMillis();
        }
    }
    @Override
    public void cleanUp() {
        robotHardware.setLiftPosition(liftPos);
    }

    @Override
    public boolean isDone() {
        return mode==mode.OUT && (System.currentTimeMillis() - outTime > 200);
    }
}
