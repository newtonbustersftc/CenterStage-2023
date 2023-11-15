package org.firstinspires.ftc.teamcode;

public class DropSpikeMarkTask implements RobotControl {
    RobotHardware robotHardware;
    long startTime;
    int step = 1;

    public DropSpikeMarkTask(RobotHardware hardware) {
        this.robotHardware = hardware;
    }

    public String toString() {
        return "DropSpikeMarkTask";
    }

    @Override
    public void prepare() {
        robotHardware.intakeDropSpike(0);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - startTime > step * 100) {
            robotHardware.intakeDropSpike(step);
            step++;
        }
    }
    @Override
    public void cleanUp() {
        robotHardware.resetIntakeMotor();
    }

    @Override
    public boolean isDone() {
        return step==18;
    }
}
