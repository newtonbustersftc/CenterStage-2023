package org.firstinspires.ftc.teamcode;

public class IntakeActionTask implements RobotControl {
    RobotHardware robotHardware;
    RobotHardware.IntakeMode mode;

    public IntakeActionTask(RobotHardware hardware, RobotHardware.IntakeMode mode) {
        this.robotHardware = hardware;
        this.mode = mode;
    }

    public String toString() {
        return "IntakeAction: " + mode;
    }

    @Override
    public void prepare() {
        switch (mode) {
            case ON :
                robotHardware.startIntake();
                break;
            case SLOW :
                robotHardware.startIntakeSlow();
                break;
            case REVERSE:
                robotHardware.reverseIntake();
                break;
            case OFF:
                robotHardware.stopIntake();
        }
    }

    @Override
    public void execute() {
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
