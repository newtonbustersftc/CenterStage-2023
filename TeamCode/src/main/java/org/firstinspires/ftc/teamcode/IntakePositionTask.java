package org.firstinspires.ftc.teamcode;

public class IntakePositionTask implements RobotControl {

    RobotHardware robotHardware;
    long startTime;
    long duration = 500;
    boolean isUp;

    public IntakePositionTask(RobotHardware hardware, boolean isUp) {
        this.robotHardware = hardware;
        this.isUp = isUp;
    }

    public String toString() {
        return "IntakePosition: " + (isUp?"UP":"DOWN");
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        if (isUp) {
            robotHardware.intakePosUp();
        }
        else{
            robotHardware.intakePosDown();
        }
        Logger.logFile("IntakePosition: " + (isUp?"UP":"DOWN"));
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
