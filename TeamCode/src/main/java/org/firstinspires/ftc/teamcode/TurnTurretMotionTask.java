package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TurnTurretMotionTask implements RobotControl {
    enum Mode { POWER_1, RAMP_DOWN, PID, DONE };
    Mode mode;
    RobotHardware robotHardware;
    DcMotorEx turretMotor;
    int powerSign = 1;
    double power;
    int targetPos;
    int currPos;
    long startTime;
    long rampDownStart;

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
            powerSign = (targetPos > currPos) ? 1 : -1;
            turretMotor.setPower(powerSign * power);
            Logger.logFile("Turret turn Motion Profile from:" + currPos + " to: " + targetPos);
        }
    }

    @Override
    public void execute() {
        currPos = robotHardware.getTurretPosition();
        if (mode == Mode.POWER_1) {
            int remain = Math.abs(targetPos - currPos);
            double velocity = robotHardware.getTurretVelocity();
            int distToStop = calcDistToStop(Math.abs(velocity));
            if (remain <= distToStop - 20) {
                Logger.logFile("Turret ramp down at " + currPos + " with velocity " + velocity);
                rampDownStart = System.currentTimeMillis();
                mode = Mode.RAMP_DOWN;
            }
        }
        else if (mode == Mode.RAMP_DOWN) {
            int remain = powerSign * (targetPos - currPos);
            if (remain < 100 || (System.currentTimeMillis() - rampDownStart)>1000) {
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(robotHardware.getRobotProfile().hardwareSpec.turretPower);
                turretMotor.setTargetPosition(targetPos);
                mode = Mode.PID;
                Logger.logFile("Turret go PID at " + currPos);
            }
            else {
                double power = Math.max(0.05, 1 - (System.currentTimeMillis() - rampDownStart) * robotHardware.getRobotProfile().hardwareSpec.turretPowerDownMs);
                double velocity = robotHardware.getTurretVelocity();
                int distToStop = calcDistToStop(Math.abs(velocity));
                power = power + (remain - distToStop)/remain * robotHardware.getRobotProfile().hardwareSpec.turretRampDownP;
                turretMotor.setPower(power * powerSign);
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

    int calcDistToStop(double velocity) {
        double timeToZero = velocity / robotHardware.getRobotProfile().hardwareSpec.turretDecelerate / 1000;   // second
        int dist = (int)(timeToZero * velocity / 2);
        return dist;
    }
}