package org.firstinspires.ftc.teamcode;

public class AutoConePlacementTask implements RobotControl {
    RobotHardware robotHardware;
    long startTime;
    enum Mode {WAIT, MOVE_TURRET, WAIT2, EXTEND_ARM}
    Mode mode;
    PoleRecognition poleRecognition;
    RobotProfile robotProfile;
    int expectedTurretPosition;

    public AutoConePlacementTask(RobotHardware hardware, RobotProfile robotProfile, PoleRecognition poleRecognition) {
        this.robotHardware = hardware;
        this.poleRecognition = poleRecognition;
        this.robotProfile = robotProfile;
    }

    public String toString() {
        return "Cone Placement " + 0;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        mode = Mode.WAIT;
    }

    @Override
    public void execute() {
        if (mode == Mode.WAIT) {
            if (System.currentTimeMillis() - startTime > 100) {
                int center = poleRecognition.getPoleCenterOnImg();
                Logger.logFile("Pole Center: " + center);
                int turretPos = robotHardware.getTurretPosition();
                if (turretPos == -1) {
                    mode = Mode.WAIT2;
                }
                expectedTurretPosition = turretPos + calculateTurretMovement(center);
                Logger.logFile("Expected Position: " + expectedTurretPosition);
                Logger.logFile("Current Position: " + turretPos);
                robotHardware.setTurretPosition(expectedTurretPosition);
                mode = Mode.MOVE_TURRET;
            }
        } else if (mode == Mode.MOVE_TURRET) {
            if (Math.abs(expectedTurretPosition - robotHardware.getTurretPosition()) <= 5) {
                mode = Mode.WAIT2;
            }
        }

    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return mode == Mode.WAIT2;
    }

    private int calculateTurretMovement(int center) {
        int index = 0;
        int[] pos = robotProfile.poleParameter.centerPosition;
        while (index < pos.length - 1 ) {
            if (center > pos[index] && center < pos[index + 1]) {
                return (5 - index) * 25 + (center - pos[index]) * 25 / (pos[index + 1] - pos[index]);
            }
            index++;
        }
        return -1;
    }
}