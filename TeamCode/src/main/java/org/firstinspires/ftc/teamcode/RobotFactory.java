package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Factory class for RobotHardware object so that we do not initialize twice and lose the motor position.
 */
public class RobotFactory {
    static RobotHardware  theRobot = null;

    static public RobotHardware getRobotHardware(HardwareMap hardwareMap,RobotProfile robotProfile){
        if(theRobot == null){
            Logger.logFile("Creating new RobotHardware Instance");
            theRobot = new RobotHardware();
            theRobot.init(hardwareMap, robotProfile);
        }
        else {
            Logger.logFile("Use existing RobotHardware Instance");
        }
        return theRobot;
    }

    static public RobotHardware getRobotHardware() {
        return theRobot;
    }

    public static void reset() {
        theRobot = null;
    }
}
