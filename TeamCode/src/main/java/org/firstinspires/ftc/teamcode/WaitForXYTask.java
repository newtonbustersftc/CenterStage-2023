package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/** Wait for the robot to be in the rectangle X,Y coordinate
 */
public class WaitForXYTask implements RobotControl {
    RobotHardware robotHardware;
    long maxDelay;
    Vector2d vecMin, vecMax;
    long timeStart;

    public WaitForXYTask(RobotHardware robotHardware, Vector2d vecMin, Vector2d vecMax, long maxDelay) {
        this.robotHardware = robotHardware;
        this.maxDelay = maxDelay;
        this.vecMin = vecMin;
        this.vecMax = vecMax;
    }

    public String toString() {
        return "WaitForXYTask " + this.vecMin + "," + this.vecMax;
    }

    @Override
    public void prepare() {
        timeStart = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    @Override
    public void cleanUp() {
    }

    @Override
    public boolean isDone() {
        Pose2d currPose = robotHardware.getLocalizer().getPoseEstimate();
        return (System.currentTimeMillis() > timeStart + maxDelay) ||
                (currPose.getX() > vecMin.getX() && currPose.getY() > vecMin.getY() &&
                        currPose.getX() < vecMax.getX() && currPose.getY() < vecMax.getY());
    }
}
