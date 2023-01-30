package org.firstinspires.ftc.teamcode;

public class ExtendArmTask implements RobotControl {
    RobotHardware robotHardware;
    double armPos;
    long startTime;
    long duration;

    public ExtendArmTask(RobotHardware hardware, double armPos) {
        this.robotHardware = hardware;
        this.armPos = armPos;
        duration = 1000;
    }

    public ExtendArmTask(RobotHardware hardware, double armPos, int duration) {
        this.robotHardware = hardware;
        this.armPos = armPos;
        this.duration = duration;
    }

    public String toString() {
        return "ExtendArmTask " + armPos;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        if (armPos==robotHardware.getExtensionPosition()) {
            duration = 0;   // we done!
        }
        robotHardware.setExtensionPosition(armPos);
        Logger.logFile("Extension Arm to: " + armPos);
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
