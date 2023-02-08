package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TurnTurretMotionTask implements RobotControl {
    enum Mode { POWER_1, GLIDE, PID, DONE };
    Mode mode;
    RobotHardware robotHardware;
    DcMotorEx turretMotor;
    double power;
    int targetPos;
    int currPos;
    long startTime;

    public TurnTurretMotionTask(RobotHardware hardware, int targetPos, double power) {
        this.robotHardware = hardware;
        this.targetPos = targetPos;
        this.power = power;
    }

    public TurnTurretMotionTask(RobotHardware hardware, int targetPos) {
        this.robotHardware = hardware;
        this.targetPos = targetPos;
        this.power = hardware.getRobotProfile().hardwareSpec.turretPower;
    }

    public String toString() {
        return "Motion Profile Turn turret to " + targetPos;
    }

    @Override
    public void prepare() {
        startTime = System.currentTimeMillis();
        turretMotor = robotHardware.turretMotor;
        currPos = robotHardware.getTurretPosition();
        if (Math.abs(targetPos - currPos) < 300) {
            mode = Mode.PID;
            robotHardware.setTurretPosition(targetPos);
            Logger.logFile("Turret PID turn from:" + currPos + " to: " + targetPos);
        }
        else {
            mode = Mode.POWER_1;
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            turretMotor.setPower((targetPos > currPos) ? power : -power);
            Logger.logFile("Turret turn Motion Profile from:" + currPos + " to: " + targetPos);
        }
    }

    @Override
    public void execute() {
        currPos = robotHardware.getTurretPosition();
        if (mode == Mode.POWER_1) {
            int remain = Math.abs(targetPos - currPos);
            double velocity = robotHardware.getTurretVelocity();
            int distToGlide = calcDistToGlide(Math.abs(velocity));
            if (remain <= distToGlide - 20) {
                Logger.logFile("Turret glide at " + currPos + " with velocity " + velocity);
                mode = Mode.GLIDE;
                turretMotor.setPower(0);
            }
        }
        else if (mode == Mode.GLIDE) {
            int remain = Math.abs(targetPos - currPos);
            if (remain < 100) {
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(0.3);
                turretMotor.setTargetPosition(targetPos);
                mode = Mode.PID;
                Logger.logFile("Turret go PID at " + currPos);
            }
        }
    }
    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        boolean done = (System.currentTimeMillis()-startTime)>100 && !robotHardware.isTurretTurning() &&
                Math.abs(currPos - targetPos)<20;
        if (done) {
            Logger.logFile("Turret turn done pos " + currPos);
        }
        return done;
    }

    int calcDistToGlide(double velocity) {
        double timeToZero = velocity / robotHardware.getRobotProfile().hardwareSpec.turretDecelerate / 1000;   // second
        int dist = (int)(timeToZero * velocity / 2);
        return dist;
    }
}