package org.firstinspires.ftc.teamcode;

public class AutoConePlacementTask implements RobotControl {
    RobotHardware robotHardware;
    long startTime;
    enum Mode {WAIT, MOVE_TURRET, WAIT2, EXTEND_ARM, DONE}
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
        poleRecognition.saveNextImg();
        mode = Mode.WAIT;
    }

    @Override
    public void execute() {
        if (mode == Mode.WAIT) {
            if (System.currentTimeMillis() - startTime > 100) {
                int center = poleRecognition.getPoleCenterOnImg();
                int width = poleRecognition.getPoleWidthOnImg();
                Logger.logFile("AutoCone 1st Pole Center: " + center + " Width: " + width);
                int turretPos = robotHardware.getTurretPosition();
                if (turretPos == -1) {
                    mode = Mode.DONE;   // we did not see a pole
                }
                expectedTurretPosition = turretPos + calculateTurretMovement(center);
                Logger.logFile("Turret at: " + turretPos + " to:" + expectedTurretPosition);
                robotHardware.setTurretPosition(expectedTurretPosition);
                mode = Mode.MOVE_TURRET;
            }
        }
        else if (mode == Mode.MOVE_TURRET) {
            if (Math.abs(expectedTurretPosition - robotHardware.getTurretPosition()) <= 5) {
                startTime = System.currentTimeMillis();
                mode = Mode.WAIT2;
                poleRecognition.saveNextImg();
            }
            // else - continue to turn
        }
        else if (mode == Mode.WAIT2) {
            if (System.currentTimeMillis() - startTime > 300) {
                int center = poleRecognition.getPoleCenterOnImg();
                int width = poleRecognition.getPoleWidthOnImg();
                Logger.logFile("AutoCone 1st Pole Center: " + center + " Width: " + width);
                double armPos = calculateArmExtension(width);
                Logger.logFile("Need arm extension: " + armPos);
                if (armPos!=-1) {
                    robotHardware.setExtensionPosition(armPos);
                }
                mode = Mode.DONE;
            }
        }
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        return mode == Mode.DONE;
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

    private double calculateArmExtension(int width) {
        // calculate the ratio
        double r = 1.0*(width - robotProfile.poleParameter.farthestPoleWidth)/
                    (robotProfile.poleParameter.closestPoleWidth-robotProfile.poleParameter.farthestPoleWidth);
        if (r>1 || r<0) {
            return -1;  // no good
        }
        return robotProfile.poleParameter.farthestArmExtension - r *
                (robotProfile.poleParameter.farthestArmExtension - robotProfile.hardwareSpec.extensionInitPos);
    }
}