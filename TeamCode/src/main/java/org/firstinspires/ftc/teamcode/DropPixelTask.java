package org.firstinspires.ftc.teamcode;

public class DropPixelTask implements RobotControl {
    RobotHardware robotHardware;
    long dropTime;
    enum Mode { OPEN, IN, LIFT}
    Mode mode;

    public DropPixelTask(RobotHardware hardware) {
        this.robotHardware = hardware;
    }

    public String toString() {
        return "DropUpTask";
    }

    @Override
    public void prepare() {
        robotHardware.grabberOpen();
        mode = Mode.OPEN;
        dropTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (mode==Mode.OPEN && (System.currentTimeMillis() - dropTime) > 200) {
            robotHardware.grabberUp();
            robotHardware.grabberIn();
            mode = Mode.IN;
        }
        else if (mode==Mode.IN && (System.currentTimeMillis() - dropTime) > 400) {
            robotHardware.setLiftPosition(0);
            mode = Mode.LIFT;
        }
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis() - dropTime > 800;
    }
}
