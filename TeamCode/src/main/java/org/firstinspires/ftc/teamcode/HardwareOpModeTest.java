package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@TeleOp(name="Hardware Test", group="Test")
//@Disabled
public class HardwareOpModeTest extends OpMode {
    RobotHardware robotHardware;
    RobotVision robotVision;
    RobotProfile robotProfile;

    Pose2d currPose;
    double grabberPos = 0.5;
    boolean grabberChange =  false;
    double fieldHeadingOffset;
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

        Logger.init();
        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);
        robotHardware.grabberOpen();
        robotHardware.calibrateGyro(telemetry);
        robotVision = robotHardware.getRobotVision();
        //robotVision.activateNavigationTarget();
        //robotHardware.getRobotVision().initWebCam("Webcam", true);  //boolean isRed

        try {
            Thread.sleep(100);
        }
        catch (Exception e) {
        }

        //robotVision.startWebcam("Webcam", null);
    }

    @Override
    public void loop() {
        robotHardware.clearBulkCache();
        robotHardware.getLocalizer().update();
        currPose = robotHardware.getLocalizer().getPoseEstimate();

        testHardware();

        telemetry.addData("Heading", Math.toDegrees(robotHardware.getGyroHeading()));
        telemetry.addData("Lift Touch", robotHardware.isLiftTouched());
        telemetry.addData("Magnetic Sensor", robotHardware.isMagneticTouched());
        telemetry.addData("Lift Position", robotHardware.getLiftPosition());
        telemetry.addData("Turret Position", robotHardware.getTurretPosition());
        telemetry.addData("Extension Position", robotHardware.extensionPos);
        telemetry.addData("Gyro", Math.toDegrees(robotHardware.getGyroHeading()));
        telemetry.addData("ConeRef", robotHardware.getConeReflection());
        telemetry.addData("Grabber", grabberPos);
        telemetry.addLine().addData("FL", robotHardware.flMotor.getCurrentPosition())
                .addData("RL:", robotHardware.rlMotor.getCurrentPosition())
                .addData("RR:", robotHardware.rrMotor.getCurrentPosition())
                .addData("FR:", robotHardware.frMotor.getCurrentPosition());
        telemetry.update();

        robotHardware.turnTurret(gamepad1.left_stick_x);

        if (gamepad1.x) { // Square
            robotHardware.grabberClose();
        } else if (gamepad1.y) { // Triangle
            robotHardware.grabberOpen();
        }

        if (gamepad1.dpad_left) {
            robotHardware.extensionExtend();
        } else if (gamepad1.dpad_right) {
            robotHardware.extensionRetract();
        }

        if (!grabberChange) {
            if (gamepad1.right_bumper) {
                grabberPos += 0.01;
                robotHardware.grabberServo.setPosition(grabberPos);
            }
            if (gamepad1.left_bumper) {
                grabberPos -= 0.01;
                robotHardware.grabberServo.setPosition(grabberPos);
            }
        }
        grabberChange = (gamepad1.left_bumper || gamepad1.right_bumper);

        robotHardware.turnOnLight(gamepad1.left_trigger>0.3);

        if (robotHardware.isLiftTouched()) {robotHardware.resetLiftPos();}

        if (robotHardware.isMagneticTouched()) {robotHardware.resetTurretPos();}

    }

    @Override
    public void stop() {
        // open the clamp to relief the grabber servo
        try {
            //robotVision.stopWebcam("Webcam");
            robotHardware.stopAll();
            Logger.logFile("DriverOpMode Test stop() called");
            Logger.flushToFile();
        } catch (Exception e) {
            Log.e("DriverOpMode", Log.getStackTraceString(e));
        }
    }

    private void testHardware() {
    }
}