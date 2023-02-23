package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.io.File;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

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
    int loop;
    boolean writeCsv = false;

    class TurretRecord {
        long ts; double power; double velocity; int position;
        public TurretRecord(long ts, double power, double velocity, int position) {
            this.ts = ts;
            this.power = power;
            this.velocity = velocity;
            this.position = position;
        }

        public String toString() {
            return "" + ts + "," + power + "," + velocity + "," + position;
        }
    }
    public static int TOTAL_REC_CNT = 1000;
    TurretRecord[] recording = new TurretRecord[TOTAL_REC_CNT];


    public TurnTurretMotionTask(RobotHardware hardware, int targetPos, double power) {
        this.robotHardware = hardware;
        this.targetPos = targetPos;
        this.power = power;
    }

    public void setWriteCsv(boolean writeCsv) {
        this.writeCsv = writeCsv;
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
        loop = 0;
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
        long currTime = System.currentTimeMillis();
        double pwr = power;
        double velocity = robotHardware.getTurretVelocity();
        if (mode == Mode.POWER_1) {
            int remain = powerSign * (targetPos - currPos);
            int distToStop = calcDistToStop(Math.abs(velocity));
            if (remain <= distToStop - 20) {
                Logger.logFile("Turret ramp down at " + currPos + " with velocity " + velocity);
                rampDownStart = currTime;
                mode = Mode.RAMP_DOWN;
            }
        }
        else if (mode == Mode.RAMP_DOWN) {
            int remain = powerSign * (targetPos - currPos);
            if (remain < 50 || (currTime - rampDownStart)>1000) {
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(robotHardware.getRobotProfile().hardwareSpec.turretPower);
                turretMotor.setTargetPosition(targetPos);
                mode = Mode.PID;
                Logger.logFile("Turret go PID at " + currPos);
            }
            else {
                pwr = power - (currTime - rampDownStart) * robotHardware.getRobotProfile().hardwareSpec.turretPowerDownMs;
                int distToStop = calcDistToStop(Math.abs(velocity));
                pwr = pwr + (remain - distToStop) * robotHardware.getRobotProfile().hardwareSpec.turretRampDownP;
                pwr = Math.max(0, pwr);
                turretMotor.setPower(pwr * powerSign);
            }
        }
        if (writeCsv && loop<TOTAL_REC_CNT) {
            recording[loop] = new TurretRecord(currTime - startTime, pwr, velocity, currPos);
            loop++;
        }
    }
    @Override
    public void cleanUp() {
        if (writeCsv) {
            String timestamp = new SimpleDateFormat("yyyyMMdd-HHmm", Locale.US).format(new Date());
            try {
                PrintWriter pw = new PrintWriter(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/turdat_" + timestamp + ".csv"));
                pw.println("ts,power,velocity,position");
                for (int i = 0; i < loop; i++) {
                    pw.println(recording[i]);
                }
                pw.flush();
                pw.close();
            } catch (Exception ex) {
                ex.printStackTrace();
            }
            try {
                Logger.flushToFile();
            } catch (Exception ex) {
            }
        }
    }

    @Override
    public boolean isDone() {
        boolean done = (System.currentTimeMillis()-startTime)>100 && !robotHardware.isTurretTurning() &&
                powerSign * (targetPos - currPos)<20;
        //done = (System.currentTimeMillis() - startTime) > 3000; // for charting purpose
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