package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;


@TeleOp(name="SOLO DriverOpMode", group="Main")
public class SoloDriverOpMode extends OpMode {
    class LastLiftExtTut {
        double extension = -1;
        int liftPos = -1;
        int tutPos = -1;
        double robHead = -1;

        public String toString() {
            return "lift:" + liftPos + " Turret:" + tutPos + " Ext:" + extension + " Heading:" + robHead;
        }
    }
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    boolean fieldMode;
    boolean isRedTeam;   // always true for current autonomous set up
    boolean leftBumperPressed = false;
    double imuAngleOffset = Math.PI/2;
    boolean liftCanChange = true;
    boolean gripperCanChange = true;
    boolean justAutoPole = false;
    boolean touching = false;
    double touchX, touchY;
    int touchTurret;
    long loopCnt;
    double currHeading;

    RobotControl currentTask = null;
    boolean safeDrive = false;

    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profileA.json"));
        }
        catch (Exception e) {
            System.out.println(e.getStackTrace());
        }
        fieldMode = true; //robot starts in field orientation

        Logger.init();
        Logger.logFile("OpMode - DriverOpMode");

        //Obtain the RobotHardware object from factory
        robotHardware = RobotFactory.getRobotHardware(hardwareMap, robotProfile);
        robotHardware.resetDriveAndEncoders();
//        robotHardware.turnUpSignalBlocker();
        robotHardware.enableManualCaching(true);
        robotHardware.clearBulkCache();
        robotHardware.getLocalizer().update();
        currHeading = robotHardware.getGyroHeading();
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(robotHardware.getHardwareMap());
        String startPosMode = prefs.getString(AutonomousOptions.START_POS_MODES_PREF, AutonomousOptions.START_POS_MODES[0]);
        isRedTeam = startPosMode.startsWith("RED");

        Logger.logFile("IMU Offset is " + Math.toDegrees(imuAngleOffset));
        Logger.logFile("Current IMU Angle " + Math.toDegrees(currHeading));
        Logger.logFile("StartPos: " + startPosMode);
        setupCombos();
        //robotHardware.grabberOpen();

        loopCnt = 0;
    }

    /**
     * Main loop for the code
     */
    @Override
    public void loop() {
        loopCnt++;
        robotHardware.clearBulkCache();
        if (loopCnt % 10==0) {
            currHeading = robotHardware.getGyroHeading();   // only use Gyro heading every 10 times
        }
        if (loopCnt % 1000==0) {
            Logger.logFile(robotHardware.getLiftMotorPos());
        }
        //Handling autonomous task loop
        if (currentTask != null) {
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
            if (gamepad1.options && gamepad1.dpad_down) {
                currentTask = new LiftResetTask(robotHardware, robotProfile);
                currentTask.prepare();
            }
        }

        handleMovement();
        handleGripper();
        handleLift();
        telemetry.addData("Heading", Math.toDegrees(currHeading));
        telemetry.update();
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
        if (fieldMode) {
            movAngle = padAngle + Math.PI / 2 - currHeading -imuAngleOffset;
        } else {
            movAngle = padAngle;
        }
        if (gamepad1.left_trigger > 0) {
            power = power / 3;
            turn = turn / 3;
        }
//        telemetry.addData("Power:", power);
//        telemetry.addData("Move Angle:", movAngle);
//        telemetry.addData("Turn:", turn);
        if (power > 0.2) {
            robotHardware.setMotorStopBrake(false);
        }
        robotHardware.mecanumDrive2(power, movAngle, turn);

        if(gamepad1.options && gamepad1.dpad_right){
            robotHardware.resetImu();
            imuAngleOffset = -Math.PI/2;
            fieldMode = true;
        }
    }

    // Use touch pad to control lift with 1 finger

    public void handleLift() {

    }

    public void handleGripper() {
    }

    /**
     * Define combo task for driver op mode
     */
    void setupCombos() {

    }
}
