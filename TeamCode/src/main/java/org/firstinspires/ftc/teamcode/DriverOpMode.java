package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

import static org.firstinspires.ftc.teamcode.AutonomousOptions.START_POS_MODES_PREF;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="Newton DriverOpMode", group="Main")
public class DriverOpMode extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    Pose2d currPose;
    double fieldHeadingOffset;

    boolean fieldMode;
    int fieldModeSign = -1;  // RED side = 1, BLUE side = -1
    boolean isRedTeam;
    boolean dpadRightPressed = false;
    boolean dpadLeftPressed = false;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;
    boolean aPressed = false;
    boolean yPressed = false;
    boolean xPressed = false;
    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;
    boolean leftTriggerPressed = false;
    boolean rightTriggerPressed = false;
    double imuAngleOffset = 0;

    // DriveThru combos
    SequentialComboTask intakeAndLift, deliverTask,  sharedHubTask;
    RobotControl currentTask = null;
    String startPosStr ;

    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        }
        catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
        fieldMode = true; //robot starts in field orientation

        Logger.init();
        //Obtain the RobotHardware object from factory
        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        //robotHardware.initRobotVision();
        //robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        //robotHardware.initLeds();   // need to init everytime
       // robotHardware.getLocalizer().setPoseEstimate(new Pose2d(0,0,0));
        //ensure lift is reset at the beginning and the end
        // Based on the Autonomous mode starting position, define the heading offset for field mode
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        if (prefs.getString(START_POS_MODES_PREF, "NONE").startsWith("RED")) {
            fieldModeSign = 1;
            isRedTeam = true;
        }
        else {
            fieldModeSign = -1;
        }
        if (prefs.getString(START_POS_MODES_PREF, "NONE").contains("DUCK")) {
            imuAngleOffset = Math.PI;
        }
        Logger.logFile("DriverOpMode: " + prefs.getString(START_POS_MODES_PREF, "NONE"));
        Logger.logFile("IMU Offset is " + Math.toDegrees(imuAngleOffset));
        Logger.logFile("Current IMU Angle " + Math.toDegrees(robotHardware.getImuHeading()));

        //robotHardware.getRobotVision().initRearCamera(isRedTeam);

        startPosStr = prefs.getString(START_POS_MODES_PREF, "NONE").contains("BLUE") ? "BLUE" : "RED";
        setupCombos();

    }

    /**
     * Main loop for the code
     */
    @Override
    public void loop() {
        //Read values from the control hub
        robotHardware.clearBulkCache();

        //currPose = new Pose2d(0,0,0);   // for now
        //Handling autonomous task loop
        if (currentTask != null) {
            robotHardware.setLed1(true);
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                currentTask.cleanUp();
                currentTask = null;
            }
            else {
                currentTask.execute();
                if (currentTask.isDone()) {
                    currentTask.cleanUp();
                    Logger.logFile("TaskComplete: " + currentTask);
                    currentTask = null;
                }
            }
        }
        else {
            robotHardware.setLed1(false);
        }

        handleMovement();

        telemetry.addData("Heading", Math.toDegrees(robotHardware.getImuHeading()));


    }

    @Override
    public void stop() {
        robotHardware.stopAll();
        try {
            Logger.logFile("DriverOpMode stop() called");
            //robotVision.deactivateNavigationTarget();
            Logger.flushToFile();
        }
        catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
    }

    /**
     * Joystick Driving Controls
     * Left bumper for slow motion
     * Left trigger to enable field mode, right trigger to enable robot-oriented mode
     */
    private void handleMovement() {
        double turn = gamepad1.right_stick_x / 2;
        double power = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double padAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) + Math.PI / 2;

        double movAngle;

        robotHardware.setLed2(fieldMode);
        if (fieldMode) {
            movAngle = padAngle + ((isRedTeam) ? Math.PI / 2 : -Math.PI / 2) - robotHardware.getImuHeading()-imuAngleOffset;
        } else {
            movAngle = padAngle;
        }
        if (gamepad1.left_trigger > 0) {
            power = power / 3;
            turn = turn / 3;
        }
        robotHardware.mecanumDrive2(power, movAngle, turn);

        // toggle field mode on/off.
        // Driver 1: dpad down - enable; dpad right - disable
        if (gamepad1.dpad_down) {
            fieldMode = true;
        } else if (gamepad1.dpad_right) {
            fieldMode = false;  //good luck driving
        }
        if(gamepad1.share){
            robotHardware.resetImu();
            imuAngleOffset = 0;
            fieldMode = true;
        }

    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {

    }
}
