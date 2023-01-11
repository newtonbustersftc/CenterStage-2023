package org.firstinspires.ftc.teamcode;

public class GrabberTask implements RobotControl {
    public enum GrabberState { OPEN, CLOSE, INIT, SAFE };

    RobotHardware robotHardware;
    long startTime;
    long duration = 500;
    GrabberState state;

    public GrabberTask(RobotHardware hardware, boolean isOpen) {
        this.robotHardware = hardware;
        if (isOpen) {
            state = GrabberState.OPEN;
        }
        else {
            state = GrabberState.CLOSE;
        }
    }

    public GrabberTask(RobotHardware hardware, GrabberState state) {
        this.robotHardware = hardware;
        this.state = state;
    }

    public String toString() {
        return "GrabberTask State: " + state;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        if (state==GrabberState.OPEN) {
            robotHardware.grabberOpen();
        }
        else if(state==GrabberState.INIT) {
            robotHardware.grabberInit();
        }
        else if (state==GrabberState.SAFE) {
            robotHardware.grabberMoveSafe();
        }
        else{
            robotHardware.grabberClose();
        }
        Logger.logFile("Grabber state: " + state);
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
