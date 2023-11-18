package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftResetTask implements RobotControl {
    enum Mode { UP, WAIT, DOWN, REST, DONE };
    Mode mode;
    RobotHardware robotHardware;
    long startTime, waitStart, restStart;
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
        robotHardware.setLiftPosition(300, 0.3);
        mode = Mode.UP;
    }

    @Override
    public void execute() {
        if (mode==Mode.UP && (System.currentTimeMillis() - startTime > 100)) {
            if (robotHardware.getLiftPosition()>300 ||
                !robotHardware.isLiftMoving()) {
                Logger.logFile("ResetLift WAIT mode");
                robotHardware.setLiftPower(0);
                mode = Mode.WAIT;
                waitStart = System.currentTimeMillis();
                robotHardware.grabberOpen();
                robotHardware.grabberUp();
                robotHardware.grabberIn();
            }
        }
        if (mode==Mode.WAIT && (System.currentTimeMillis() - waitStart > 200)) {
            Logger.logFile("ResetLift DOWN mode");
            mode = Mode.DOWN;
            robotHardware.setLiftPosition(-5000, 0.1);
        }
        if (mode==Mode.DOWN && (System.currentTimeMillis() - waitStart> 500)) {
            if (!robotHardware.isLiftMoving() || (System.currentTimeMillis() - waitStart>6000)) {
                Logger.logFile("ResetLift REST mode");
                robotHardware.setLiftPower(0.05);
                mode = Mode.REST;
                restStart = System.currentTimeMillis();
            }
        }
        if (mode==Mode.REST) {
            if (System.currentTimeMillis() - restStart >200) {
                Logger.logFile("ResetLift DONE mode");
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