package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.NBMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

/**
 * sophia 11-6-21 14:32
 * marianna 11-6-21 09:00-14:30
 * Reuse keystone pid mecanum movements for our first meet.
 */

public class AutonomousTaskBuilder {
    RobotProfile robotProfile;
    RobotHardware robotHardware;
    ArrayList<RobotControl> taskList = new ArrayList<RobotControl>();
    DriverOptions driverOptions;
    NBMecanumDrive drive;
    Pose2d startPos = new Pose2d();
    int start_delay;
    String startingPositionModes;
    RobotVision.AutonomousGoal goal;

    public AutonomousTaskBuilder(DriverOptions driverOptions, RobotHardware robotHardware, RobotProfile robotProfile, RobotVision.AutonomousGoal goal) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;

        this.driverOptions = driverOptions;
        this.start_delay = driverOptions.getStartDelay();
        this.startingPositionModes = driverOptions.getStartingPositionModes();
        this.goal = goal;
        this.robotProfile = robotProfile;
    }

    public ArrayList<RobotControl> buildTaskList(RobotVision.AutonomousGoal goal) {
        if (start_delay>0) {
            taskList.add(new RobotSleep(start_delay*1000));
        }

        // buildDuckTasks(startingPositionModes);
        return taskList;
    }
}