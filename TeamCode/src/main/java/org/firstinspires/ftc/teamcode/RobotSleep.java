package org.firstinspires.ftc.teamcode;

/**
 * 2019.10.26
 * Athena Z.
 */

public class RobotSleep implements RobotControl {
    long timeSleep;
    long timeStart;
    long timeNow;
    String memo = null;
    public RobotSleep(int timeSleep) {
        this.timeSleep = timeSleep;
    }

    public RobotSleep(int timeSleep, String memo) {
        this.timeSleep = timeSleep;
        this.memo = memo;
    }

    public String toString() {
        return "Sleep for " + timeSleep + (memo!=null?" " + memo:"");
    }

    @Override
    public void prepare() {
        timeStart = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        timeNow = System.currentTimeMillis();
    }

    @Override
    public void cleanUp() {

    }

    @Override
    public boolean isDone() {
        return timeNow > timeStart + timeSleep;
    }
}
