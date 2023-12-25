package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/** Use for splinkMoveTask, so another task can update the target pose */
public class Pose2dRef {
    Pose2d pose2d;

    public Pose2dRef(Pose2d pose2d) {
        this.pose2d = pose2d;
    }

    public Pose2d getPose2d() {
        return pose2d;
    }

    public void setPose2d(Pose2d pose2d) {
        this.pose2d = pose2d;
    }

}
