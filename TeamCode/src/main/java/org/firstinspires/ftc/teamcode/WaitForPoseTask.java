package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class WaitForPoseTask implements RobotControl {
    Localizer localizer;
    Pose2d pos1, pos2;
    boolean isDone = false;
    Pose2d currPose;

    public WaitForPoseTask(Localizer localizer, Pose2d low, Pose2d high) {
        this.localizer = localizer;
        this.pos1 = low;
        this.pos2 = high;
    }

    public String toString() {
        return "WaitForPoseTask: " + pos1 + " to " + pos2;
    }

    @Override
    public void prepare() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void cleanUp() {
        Logger.logFile("WaitForPoseTask is done. current pose:"+ currPose);
    }

    // isDone only when the navigator x,y,h all within the range of pos1 and pos2
    @Override
    public boolean isDone() {
        double ratioX = 0.5;        // if put in same value between low/high, ignore this compare
        double ratioY = 0.5;        // if put in same value between low/high, ignore this compare
        double ratioA = 0.5;        // if put in same value between low/high, ignore this compare

        currPose = localizer.getPoseEstimate();

        if (pos1.getX() != pos2.getX()) {
            ratioX = (currPose.getX() - pos1.getX())/(pos2.getX() - pos1.getX());
        }
        if (pos1.getY() != pos2.getY()) {
            ratioY = (currPose.getY() - pos1.getY())/(pos2.getY() - pos1.getY());
        }
        if (pos1.getHeading() != pos2.getHeading()) {
            ratioA = (currPose.getHeading() - pos1.getHeading())/(pos2.getHeading() - pos1.getHeading());
        }
        return ratioX>0 && ratioX<1.0 && ratioY>0 && ratioY<1.0 && ratioA>0 && ratioA<1.0;
    }
}
