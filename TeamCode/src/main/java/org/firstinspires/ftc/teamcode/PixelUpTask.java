package org.firstinspires.ftc.teamcode;

public class PixelUpTask implements RobotControl {
    RobotHardware robotHardware;
    long outTime;
    int liftPos;
    boolean grabberOut = false;

    public PixelUpTask(RobotHardware hardware, int liftPos) {
        this.robotHardware = hardware;
        this.liftPos = liftPos;
    }

    public String toString() {
        return "PixelUpTask liftPos: " + liftPos;
    }

    @Override
    public void prepare() {
        robotHardware.setLiftPosition(liftPos);
        grabberOut = false;
    }

    @Override
    public void execute() {
        if (!grabberOut &&
                robotHardware.getLiftPosition() > robotHardware.getRobotProfile().hardwareSpec.liftOutMin) {
            robotHardware.grabberOut();
            grabberOut = true;
            outTime = System.currentTimeMillis();
        }
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return grabberOut && (System.currentTimeMillis() - outTime > 200);
    }
}
