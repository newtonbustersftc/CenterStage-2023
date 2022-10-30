package org.firstinspires.ftc.teamcode;

public class GrabberTask implements RobotControl {
    RobotHardware robotHardware;
    boolean isOpen;
    long startTime;
    long duration = 500;

    public GrabberTask(RobotHardware hardware, boolean isOpen) {
        this.robotHardware = hardware;
        this.isOpen = isOpen;
    }

    public String toString() {
        return "GrabberTask open:" + isOpen;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        if (isOpen) {
            robotHardware.grabberOpen();
        }
        else {
            robotHardware.grabberClose();
        }
        Logger.logFile("Grabber open: " + isOpen);
    }

    @Override
    public void execute() {
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis()-startTime > duration;
    }
}
