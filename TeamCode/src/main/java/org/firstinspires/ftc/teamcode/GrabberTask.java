package org.firstinspires.ftc.teamcode;

public class GrabberTask implements RobotControl {
    RobotHardware robotHardware;
    boolean isOpen;
    long startTime;
    long duration = 500;
    double position;

    public GrabberTask(RobotHardware hardware, boolean isOpen) {
        this.robotHardware = hardware;
        this.isOpen = isOpen;
    }

    public GrabberTask(RobotHardware hardware, double position) {
        this.robotHardware = hardware;
        this.position = position;
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
        else if(!isOpen && position>0) {
            robotHardware.grabberInit();
        }
        else{
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
