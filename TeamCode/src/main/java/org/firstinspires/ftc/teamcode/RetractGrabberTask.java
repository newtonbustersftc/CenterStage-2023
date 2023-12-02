package org.firstinspires.ftc.teamcode;

public class RetractGrabberTask implements RobotControl {
    RobotHardware robotHardware;
    long startTime;
    enum Mode { OPEN, IN, DONE}
    Mode mode;

    public RetractGrabberTask(RobotHardware hardware) {
        this.robotHardware = hardware;
    }

    public String toString() {
        return "RetractGrabberTask";
    }

    @Override
    public void prepare() {
        robotHardware.grabberPreDrop();
        robotHardware.grabberUp();
        robotHardware.grabberOpen();
        mode = RetractGrabberTask.Mode.OPEN;
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (mode== Mode.OPEN && (System.currentTimeMillis() - startTime) > 200) {
            robotHardware.grabberIn();
            mode = Mode.IN;
        }
        else if (mode== Mode.IN && (System.currentTimeMillis() - startTime) > 400) {
            robotHardware.setLiftPosition(0);
            mode = Mode.DONE;
        }
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return mode==Mode.DONE;
    }
}
