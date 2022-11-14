package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="LiftPowerProfile", group="Test")
public class LiftPowerProfileOpMode extends LinearOpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    public void initRobot() {
        try {
            robotProfile = RobotProfile.loadFromFile();
        }
        catch (Exception e) {
            RobotLog.e("RobotProfile reading exception" + e);
        }
        Logger.init();
        RobotFactory.reset();
        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        Logger.logFile("Init completed");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        robotHardware.grabberOpen();
        double p = 0.5;
        Logger.logFile("Power,TimeMs,MaxVelocity");
        resetLiftZeroPosition();
        robotHardware.grabberClose();
        waitForStart();
        while (p<=1.01) {
            Thread.sleep(3000);

            double maxV = 0;
            // Testing lift
            long startTime = System.currentTimeMillis();
            robotHardware.setLiftPositionUnsafe(robotProfile.hardwareSpec.liftMax-100, p);
            while (robotHardware.getLiftPosition()<robotProfile.hardwareSpec.liftMax-110) {
                double v = robotHardware.getLiftVelocity();
                if (v>maxV) {
                    maxV = v;
                }
                Thread.yield();
            }
            long currTime = System.currentTimeMillis();
            Logger.logFile("" + p + "," + (currTime - startTime) + "," + maxV);
            p = p + 0.05;
            resetLiftZeroPosition();
        }
        Logger.logFile("Done with lift power profile test");
        Logger.flushToFile();
    }

    void resetLiftZeroPosition() {
        long currTime = System.currentTimeMillis();
        while (!robotHardware.isLiftTouched() && (System.currentTimeMillis() - currTime)<10000) {
            int currPos = robotHardware.getLiftPosition();
            robotHardware.setLiftPositionUnsafe(currPos - 2000, 0.2);
        }
        robotHardware.resetLiftPos();
    }
}
