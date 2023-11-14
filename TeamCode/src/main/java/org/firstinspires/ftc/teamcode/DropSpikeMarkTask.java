package org.firstinspires.ftc.teamcode;

public class DropSpikeMarkTask implements RobotControl {
    RobotHardware robotHardware;
    long startTime;

    public DropSpikeMarkTask(RobotHardware hardware) {
        this.robotHardware = hardware;
    }

    public String toString() {
        return "DropSpikeMarkTask";
    }

    @Override
    public void prepare() {
        robotHardware.intakeDropSpike();
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }
    @Override
    public void cleanUp() {
        robotHardware.resetIntakeMotor();
    }

    @Override
    public boolean isDone() {
        return System.currentTimeMillis() - startTime > 1000;
    }
}
