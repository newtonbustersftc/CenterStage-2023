package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftResetTask implements RobotControl {
    enum Mode { UP, WAIT, DOWN, DONE };
    Mode mode;
    RobotHardware robotHardware;
    long startTime, waitStart;
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
        robotHardware.resetLiftPos();
        robotHardware.setLiftPosition(profile.hardwareSpec.liftOutMin+100, 0.3);
        mode = Mode.UP;
    }

    @Override
    public void execute() {
        if (mode==Mode.UP && (System.currentTimeMillis() - startTime > 100)) {
            if (robotHardware.getLiftPosition()>profile.hardwareSpec.liftOutMin ||
                !robotHardware.isLiftMoving()) {
                robotHardware.setLiftPower(0);
                mode = Mode.WAIT;
                waitStart = System.currentTimeMillis();
                robotHardware.grabberOpen();
                robotHardware.grabberUp();
                robotHardware.grabberIn();
            }
        }
        if (mode==Mode.WAIT && (System.currentTimeMillis() - waitStart > 200)) {
            mode = Mode.DOWN;
            robotHardware.setLiftPosition(-5000, 0.3);
        }
        if (mode==Mode.DOWN && (System.currentTimeMillis() - waitStart> 300)) {
            if (!robotHardware.isLiftMoving()) {
                robotHardware.resetLiftPos();
                robotHardware.setLiftPower(0);
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