package org.firstinspires.ftc.teamcode;

public class SmartIntakeActionTask implements RobotControl {
    RobotHardware robotHardware;
    enum Mode { INTAKE, ROLLUP, REVERSE }
    long maxTime;
    long startTime, reverseTime;
    int count;
    Mode mode;

    public SmartIntakeActionTask(RobotHardware hardware, long maxTime) {
        this.robotHardware = hardware;
        this.maxTime = maxTime;
    }

    public String toString() {
        return "SmartIntakeAction for " + count;
    }

    @Override
    public void prepare() {
        count = robotHardware.countTrayPixel();
        startTime = System.currentTimeMillis();
        Logger.logFile("SmartIntake prepare, current count:" + count);
        if (count<2) {
            robotHardware.startIntake();
            mode = Mode.INTAKE;
        }
        else {
            mode = Mode.REVERSE;
            reverseTime = System.currentTimeMillis();
        }
    }

    @Override
    public void execute() {
        long currTime = System.currentTimeMillis();
        if (mode==Mode.INTAKE) {
            if (robotHardware.isIntakeTouched()) {
                robotHardware.intakeRollupOnly();
                mode = Mode.ROLLUP;
            }
        }
        else if (mode==Mode.ROLLUP) {
            if (robotHardware.countTrayPixel()>count) {
                count = robotHardware.countTrayPixel();
                Logger.logFile("SmartIntake prepare, new count:" + count);
                if (count<2) {
                    mode = Mode.INTAKE;
                    robotHardware.startIntake();
                }
                else {
                    mode = Mode.REVERSE;
                    robotHardware.reverseIntake();
                    reverseTime = currTime;
                    return;
                }
            }
        }
        if (currTime-startTime > maxTime) {
            mode = Mode.REVERSE;
            reverseTime = currTime;
            robotHardware.reverseIntake();
        }
    }
    @Override
    public void cleanUp() {
        robotHardware.stopIntake();
    }

    @Override
    public boolean isDone() {
        return (mode==Mode.REVERSE && (System.currentTimeMillis()-startTime>250));
    }
}
