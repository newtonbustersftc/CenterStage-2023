package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.util.Locale;

@TeleOp(name="DriverOpMode Test", group="Test")
//@Disabled
public class DriverOpModeTest extends OpMode {
    RobotHardware robotHardware;
    RobotVision robotVision;
    boolean fieldMode;
    boolean isRedTeam;
    RobotProfile robotProfile;

    Pose2d currPose;
    double fieldHeadingOffset;
    boolean upPressed = false;
    boolean downPressed = false;
    double flapServoPos = 0.3;
    double lidServoPos = 0.45;
    boolean rightPressed = false;
    boolean leftPressed = false;
    boolean yPressed = false;
    boolean xPressed = false;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean lightOn = false;
    RobotControl currentTask = null;

    @Override
    public void init() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
            Logger.logFile("Exception " + e);
            e.printStackTrace();
        }
        SharedPreferences prefs = AutonomousOptions.getSharedPrefs(hardwareMap);
        String startPosStr = prefs.getString("starting position", "none");
        Logger.logFile("StartPos:" + startPosStr);
        isRedTeam = startPosStr.toUpperCase(Locale.ROOT).startsWith("RED");

        fieldMode = true;
        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        robotVision.initRearCamera(isRedTeam);  //boolean isRed
        try {
            Thread.sleep(100);
        }
        catch (Exception e) {
        }

        robotVision.startRearCamera();
        robotHardware.initLeds();   // need to init everytime
    }

    @Override
    public void loop() {
        robotHardware.clearBulkCache();
        robotHardware.getLocalizer().update();
        currPose = robotHardware.getLocalizer().getPoseEstimate();

        handleMovement();
        // field mode or not
        if (gamepad1.left_trigger > 0) {
            fieldMode = true;
            fieldHeadingOffset = currPose.getHeading();
        } else if (gamepad1.right_trigger > 0) {
            fieldMode = false;  //good luck driving
        }
        testHardware();

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
    }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            robotVision.stopRearCamera();
            robotHardware.stopAll();
            Logger.logFile("DriverOpMode Test stop() called");
            Logger.flushToFile();
        } catch (Exception e) {
            Log.e("DriverOpMode", Log.getStackTraceString(e));
        }

    }

    private void handleMovement() {
        double turn = gamepad1.right_stick_x/4;
        double power = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double padAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) + Math.PI/2;

        double movAngle;
        if (fieldMode) {
            movAngle = padAngle+((isRedTeam)?Math.PI/2:-Math.PI/2) - currPose.getHeading();
        }
        else {
            movAngle = padAngle;
        }
        if (gamepad1.left_bumper) {
            power = power/3;
            turn = turn/3;
        }
        robotHardware.mecanumDrive2(power, movAngle, turn);
//       telemetry.addData("Stick: ", Math.toDegrees(padAngle));
//       telemetry.addData("Move: ", Math.toDegrees(movAngle));
//       telemetry.addData("Power:", power);
        telemetry.addData("Move",  "a:" + Math.toDegrees(padAngle));
        telemetry.addData("Stick", "x:" + gamepad1.left_stick_x + " y:" + gamepad1.left_stick_y);
        telemetry.addData("Mode: ", (fieldMode)?"Field":"Robot");

       // toggle field mode on/off.
       // Driver 1: left trigger - enable; right trigger - disable
        if (gamepad1.left_trigger > 0) {
            fieldMode = true;
        } else if (gamepad1.right_trigger > 0) {
            fieldMode = false;  //good luck driving
        }
        telemetry.addData("Pose:", robotHardware.getLocalizer().getPoseEstimate());
    }

    private void testHardware() {
    }
}