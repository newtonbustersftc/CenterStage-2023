package org.firstinspires.ftc.teamcode;

public class DropPixelTask implements RobotControl {
    RobotHardware robotHardware;
    long dropTime;
    boolean grabberOut;

    public DropPixelTask(RobotHardware hardware) {
        this.robotHardware = hardware;
    }

    public String toString() {
        return "DropUpTask";
    }

    @Override
    public void prepare() {
        robotHardware.grabberOpen();
        dropTime = System.currentTimeMillis();
        grabberOut = true;
    }

    @Override
    public void execute() {
        if (grabberOut && (System.currentTimeMillis() - dropTime) > 200) {
            robotHardware.grabberUp();
            robotHardware.grabberIn();
            robotHardware.setLiftPosition(0);
            grabberOut = false;
        }
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis() - dropTime > 500;
    }
}
