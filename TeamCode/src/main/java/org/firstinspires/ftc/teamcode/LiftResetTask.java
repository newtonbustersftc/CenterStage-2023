package org.firstinspires.ftc.teamcode;

public class LiftResetTask implements RobotControl {
    enum Mode { DOWN, UP, DONE };
    Mode mode;
    RobotHardware robotHardware;
    long startTime;
    RobotProfile profile;

    public LiftResetTask(RobotHardware hardware, RobotProfile profile) {
        this.robotHardware = hardware;
        this.profile = profile;
    }

    public String toString() {
        return "Lift reset task";
    }

    @Override
    public void prepare() {
        Logger.logFile("Resetting Lift Position");
        startTime = System.currentTimeMillis();
        robotHardware.setExtensionPosition(profile.hardwareSpec.extensionDriverMin);
        int currPos = robotHardware.getLiftPosition();
        robotHardware.setLiftPositionUnsafe(currPos - 5000, 0.3);
        mode = Mode.DOWN;
    }

    @Override
    public void execute() {
        if (mode==Mode.DOWN) {
            if (robotHardware.isLiftTouched()) {
                robotHardware.resetLiftPos();
                robotHardware.setLiftPositionUnsafe(100, 0.2);
                mode = Mode.UP;
            }
        }
        else {
            if (!robotHardware.isLiftTouched()) {
                robotHardware.resetLiftPos();
                robotHardware.setLiftPosition(0);
                mode = Mode.DONE;
            }
        }
    }

    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return mode==Mode.DONE || (System.currentTimeMillis()-startTime)>5000;
    }
}