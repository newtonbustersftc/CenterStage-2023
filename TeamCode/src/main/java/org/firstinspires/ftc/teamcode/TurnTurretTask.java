package org.firstinspires.ftc.teamcode;

public class TurnTurretTask implements RobotControl {
        RobotHardware robotHardware;
        int turretPos;
        int currPos;
        long startTime;

    public TurnTurretTask(RobotHardware hardware, int turretPos) {
        this.robotHardware = hardware;
        this.turretPos = turretPos;
    }

    public String toString() {
        return "Turn turret to " + turretPos;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        currPos = robotHardware.getTurretPosition();
        robotHardware.setTurretPosition(turretPos);
        Logger.logFile("Turret turn to: " + turretPos);
    }

    @Override
    public void execute() {
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return (System.currentTimeMillis()-startTime)>100 && !robotHardware.isTurretTurning();
    }
}