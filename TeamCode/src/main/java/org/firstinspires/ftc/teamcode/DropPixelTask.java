package org.firstinspires.ftc.teamcode;

public class DropPixelTask implements RobotControl {
    RobotHardware robotHardware;
    RobotProfile profile;
    long dropTime, inTime;
    enum Mode { APPROACH, OPEN, DEPART, IN, DONE, NON_ACTION}
    Mode mode;

    public DropPixelTask(RobotHardware hardware) {
        this.robotHardware = hardware;
        this.profile = hardware.getRobotProfile();
    }

    public String toString() {
        return "DropPixelTask";
    }

    @Override
    public void prepare() {
        mode = Mode.APPROACH;
    }

    @Override
    public void execute() {
        if (mode==Mode.APPROACH) {
            // use distance sensor and P(id) to reach desired distance
            double dLeft = robotHardware.getDistanceSensorLeft();
            double dRight = robotHardware.getDistanceSensorRight();
            // when both side too far, this drop will not happen
            if (dLeft > profile.hardwareSpec.autoDropMaxDist + profile.hardwareSpec.leftOffsetDist &&
                dRight > profile.hardwareSpec.autoDropMaxDist) {
                mode = Mode.NON_ACTION;
                Logger.logFile("DropPixel too far, Left: " + dLeft + " Right: " + dRight);
            }
            else if (dLeft > profile.hardwareSpec.autoDropMaxDist + profile.hardwareSpec.leftOffsetDist &&
                    dRight < profile.hardwareSpec.dropPixelDist ||
                    dRight > profile.hardwareSpec.autoDropMaxDist &&
                            dLeft < profile.hardwareSpec.dropPixelDist + profile.hardwareSpec.leftOffsetDist ) {
                // when only one side is too far, one side is in distance -> Corner situation
                mode = Mode.OPEN;
                Logger.logFile("DropPixel corner situation, Left: " + dLeft + " Right: " + dRight);
            }
            else {
                if (dLeft < profile.hardwareSpec.dropPixelDist + profile.hardwareSpec.leftOffsetDist &&
                        dRight < profile.hardwareSpec.dropPixelDist) {
                    mode = Mode.OPEN;
                    Logger.logFile("DropPixel Close enough, Left: " + dLeft + " Right: " + dRight);
                } else {
                    boolean isStop = powerMotorByDist(dLeft, profile.hardwareSpec.dropPixelDist + profile.hardwareSpec.leftOffsetDist, dRight, profile.hardwareSpec.dropPixelDist);
                    if (isStop) {
                        mode = Mode.OPEN;
                        Logger.logFile("DropPixel Power Min, Left: " + dLeft + " Right: " + dRight);
                    }
                }
            }
            if (mode==Mode.OPEN) {
                robotHardware.setMotorPower(0, 0, 0, 0);
                robotHardware.grabberOpen();
                robotHardware.grabberPreDrop();
                dropTime = System.currentTimeMillis();
            }
        }
        if (mode==Mode.OPEN && (System.currentTimeMillis() - dropTime) > 200) {
            mode = Mode.DEPART;
        }
        if (mode==Mode.DEPART) {
            boolean isStop = false;
            double dLeft = robotHardware.getDistanceSensorLeft();
            double dRight = robotHardware.getDistanceSensorRight();
            if (dLeft > profile.hardwareSpec.afterDropDist+profile.hardwareSpec.leftOffsetDist ||
                dRight > profile.hardwareSpec.afterDropDist) {
                robotHardware.setMotorPower(0,0,0,0);
                isStop = true;
            }
            else {
                isStop = powerMotorByDist(dLeft, profile.hardwareSpec.afterDropDist + profile.hardwareSpec.leftOffsetDist, dRight, profile.hardwareSpec.afterDropDist);
            }
            if (isStop) {
                robotHardware.grabberUp();
                robotHardware.grabberIn();
                mode = Mode.IN;
                inTime = System.currentTimeMillis();
            }
        }
        else if (mode==Mode.IN && (System.currentTimeMillis() - inTime) > 200) {
            robotHardware.setLiftPosition(0);
            mode = Mode.DONE;
        }
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return mode==Mode.DONE || mode== Mode.NON_ACTION;
    }

    public Mode getMode() {
        return mode;
    }

    // Stop when close enough that power to motor become small
    boolean powerMotorByDist(double dLeft, double leftTarget, double dRight, double rightTarget) {
        double leftPower = (dLeft - leftTarget) * profile.hardwareSpec.autoDropP;
        double rightPower = (dRight - rightTarget) * profile.hardwareSpec.autoDropP;
        double maxAbsPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        boolean stop = false;
        if (maxAbsPower > profile.hardwareSpec.autoDropMaxPower) {
            leftPower = leftPower / maxAbsPower * profile.hardwareSpec.autoDropMaxPower;
            rightPower = rightPower / maxAbsPower * profile.hardwareSpec.autoDropMaxPower;
        }
        else if (maxAbsPower < profile.hardwareSpec.autoDropMinPower) {
            leftPower = rightPower = 0;
            stop = true;
        }
        //Logger.logFile("dleft:" + dLeft + " dRight:" + dRight + " pwrL:" + leftPower + " pwrR:" + rightPower);
        robotHardware.setMotorPower(leftPower, rightPower, leftPower, rightPower);
        return stop;
    }
}
